# =====================================================================
#  Daidalos - launcher behind the "Daidalos diagnostika" desktop shortcut
#
#  Installed by daidalos_setup.ps1 into %LOCALAPPDATA%\Daidalos and run
#  from there. On every start:
#    1. updates the Daidalos repository and LandSim_Light to the newest
#       origin/main (git, no ZIP). Local edits to tracked files are put
#       aside with "git stash" first, never thrown away;
#    2. once a day updates Python 3.12 itself (winget);
#    3. keeps a private Python environment in %LOCALAPPDATA%\Daidalos\venv
#       and upgrades every package the GUI and LandSim need;
#    4. starts the diagnostic GUI.
#  Without internet it skips 1-3 and starts what is there.
#  Log: %LOCALAPPDATA%\Daidalos\launch.log
#
#  ASCII only on purpose: Windows PowerShell 5.1 misreads UTF-8 without BOM.
# =====================================================================
$ErrorActionPreference = "Continue"
$AppDir = Join-Path $env:LOCALAPPDATA "Daidalos"
$Venv   = Join-Path $AppDir "venv"
$VPy    = Join-Path $Venv "Scripts\python.exe"
$VPyW   = Join-Path $Venv "Scripts\pythonw.exe"
$Stamp  = Join-Path $AppDir "last_python_upgrade.txt"
try { Start-Transcript -Path (Join-Path $AppDir "launch.log") -Force | Out-Null } catch {}
$Host.UI.RawUI.WindowTitle = "Daidalos - aktualizace"

function Say($msg)  { Write-Host ">> $msg" -ForegroundColor Cyan }
function Warn($msg) { Write-Host "!! $msg" -ForegroundColor Yellow }
function Have($cmd) { return [bool](Get-Command $cmd -ErrorAction SilentlyContinue) }
function Fail($msg) {
    Write-Host "CHYBA: $msg" -ForegroundColor Red
    try { Stop-Transcript | Out-Null } catch {}
    Read-Host "Enter = zavrit"
    exit 1
}

$m = [Environment]::GetEnvironmentVariable("Path", "Machine")
$u = [Environment]::GetEnvironmentVariable("Path", "User")
$env:Path = "$m;$u"

$cfgFile = Join-Path $AppDir "config.json"
if (-not (Test-Path $cfgFile)) { Fail "Chybi $cfgFile - spustte nejdriv daidalos_setup.ps1." }
$cfg = Get-Content $cfgFile -Raw | ConvertFrom-Json
$Repo = $cfg.repo; $LandSim = $cfg.landsim
if (-not (Test-Path (Join-Path $Repo ".git"))) { Fail "$Repo neni git klon - spustte znovu daidalos_setup.ps1." }

# ---- 1. repositories ------------------------------------------------------
$online = $true
function Update-Repo($dir) {
    if (-not (Test-Path (Join-Path $dir ".git"))) { Warn "$dir neni git klon, preskakuji."; return }
    Say "Aktualizuji $dir"
    git -C $dir fetch --prune origin
    if ($LASTEXITCODE -ne 0) { Warn "Nelze stahnout (offline?) - pouzivam to, co je na disku."; $script:online = $false; return }

    $dirty = git -C $dir status --porcelain --untracked-files=no
    if ($dirty) {
        $msg = "auto-zaloha pred aktualizaci $(Get-Date -Format 'yyyy-MM-dd HH:mm')"
        git -C $dir stash push -m $msg | Out-Null
        Warn "Upravene soubory byly odlozeny do 'git stash' ($msg). Zpet: git stash pop"
    }
    $br = (git -C $dir rev-parse --abbrev-ref HEAD).Trim()
    if ($br -ne "main") { git -C $dir checkout -q main 2>$null; if ($LASTEXITCODE -ne 0) { git -C $dir checkout -q -B main origin/main } }
    git -C $dir merge --ff-only -q origin/main
    if ($LASTEXITCODE -ne 0) {
        $bak = "zaloha-$(Get-Date -Format 'yyyyMMdd-HHmm')"
        git -C $dir branch $bak | Out-Null
        git -C $dir reset -q --hard origin/main
        Warn "Lokalni commity odlozeny do vetve '$bak', slozka je ted presne origin/main."
    }
    Say ("   " + (git -C $dir log -1 --format="%h %ad %s" --date=short))
}

if (Have "git") {
    Update-Repo $Repo
    if ($online) {
        if (-not (Test-Path (Join-Path $LandSim ".git"))) {
            if (-not (Test-Path $LandSim)) {
                Say "Stahuji LandSim_Light -> $LandSim"
                git clone https://github.com/Tomasraketak/LandSim_Light.git $LandSim
            } else { Warn "$LandSim existuje, ale neni git klon - neaktualizuji ho." }
        } else { Update-Repo $LandSim }
    }
} else { Warn "Git neni nainstalovany - kod se neaktualizuje (spustte daidalos_setup.ps1)." }

# The launcher and the icon update themselves for the NEXT start.
$win = Join-Path $Repo "tools\windows"
foreach ($f in "daidalos_launch.ps1", "daidalos.ico") {
    $src = Join-Path $win $f
    if (Test-Path $src) { Copy-Item -Force $src (Join-Path $AppDir $f) -ErrorAction SilentlyContinue }
}

# ---- 2. Python itself, once a day --------------------------------------------
$today = Get-Date -Format "yyyy-MM-dd"
$last  = if (Test-Path $Stamp) { (Get-Content $Stamp -Raw).Trim() } else { "" }
if ($online -and (Have "winget") -and $last -ne $today) {
    Say "Kontroluji novou verzi Pythonu 3.12 (jednou denne)..."
    winget upgrade -e --id Python.Python.3.12 --silent --accept-package-agreements --accept-source-agreements --disable-interactivity | Out-Null
    Set-Content -Encoding ASCII $Stamp $today
}

function Find-Python312 {
    if (Have "py") {
        $p = & py -3.12 -c "import sys; print(sys.executable)" 2>$null
        if ($LASTEXITCODE -eq 0 -and $p -and (Test-Path $p.Trim())) { return $p.Trim() }
    }
    foreach ($c in (Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"),
                   (Join-Path $env:ProgramFiles "Python312\python.exe")) {
        if (Test-Path $c) { return $c }
    }
    return $null
}

# ---- 3. the private environment and the packages ---------------------------------
$base = Find-Python312
$ok = (Test-Path $VPy) -and ((& $VPy -c "import sys; print(sys.version_info[:2]==(3,12))" 2>$null) -eq "True")
if (-not $ok) {
    if (-not $base) { Fail "Python 3.12 nenalezen - spustte daidalos_setup.ps1." }
    Say "Vytvarim prostredi Pythonu v $Venv"
    & $base -m venv --clear $Venv
    if ($LASTEXITCODE -ne 0) { Fail "python -m venv selhal." }
}

if ($online) {
    Say "Aktualizuji balicky (pip)..."
    & $VPy -m pip install --upgrade --disable-pip-version-check -q pip
    & $VPy -m pip install --upgrade --upgrade-strategy eager --disable-pip-version-check -q `
        -r (Join-Path $Repo "Ariadna_V2\Diagnostics\requirements.txt")
    if ($LASTEXITCODE -ne 0) { Warn "Nektere balicky diagnostiky se nepodarilo aktualizovat." }
    $lreq = Join-Path $LandSim "requirements.txt"
    if (Test-Path $lreq) {
        & $VPy -m pip install --upgrade --upgrade-strategy eager --disable-pip-version-check -q -r $lreq
        if ($LASTEXITCODE -ne 0) {
            # numba is optional and sometimes lags behind a new Python
            Warn "Plne pozadavky LandSim selhaly, instaluji aspon numpy + matplotlib."
            & $VPy -m pip install --upgrade --disable-pip-version-check -q numpy matplotlib
        }
    }
}

# ---- 4. start the GUI ---------------------------------------------------------------
$diag = Join-Path $Repo "Ariadna_V2\Diagnostics\daidalos_diag.py"
if (-not (Test-Path $diag)) { Fail "Nenalezeno: $diag" }
& $VPy -c "import PyQt6, serial, numpy" 2>$null
if ($LASTEXITCODE -ne 0) { Fail "V prostredi chybi PyQt6 / pyserial / numpy a nejde je stahnout (offline?)." }

$env:DAIDALOS_LANDSIM = $LandSim       # the ignition tab's LandSim_Light folder
Say "Spoustim diagnostiku..."
Start-Process -FilePath $VPyW -ArgumentList "`"$diag`"" -WorkingDirectory (Split-Path $diag)
try { Stop-Transcript | Out-Null } catch {}
Start-Sleep -Seconds 2
