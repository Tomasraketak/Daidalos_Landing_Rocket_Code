# =====================================================================
#  Daidalos - one-time setup on Windows
#
#  Run from cmd (one line):
#    powershell -NoProfile -ExecutionPolicy Bypass -Command "irm https://raw.githubusercontent.com/Tomasraketak/Daidalos_Landing_Rocket_Code/main/tools/windows/daidalos_setup.ps1 | iex"
#
#  What it does (safe to run again at any time):
#    1. installs Git and Python 3.12 with winget, if they are missing;
#    2. puts the whole repository into  <Desktop>\Daidalos code  as a git
#       clone (no ZIP). An existing folder that is NOT a git clone is kept,
#       renamed to "Daidalos code (stara kopie <date>)";
#    3. clones LandSim_Light next to it (<Desktop>\LandSim_Light);
#    4. creates the "Daidalos diagnostika" shortcut on the desktop with the
#       rocket icon. The shortcut runs daidalos_launch.ps1, which on every
#       start updates both repositories, Python and the packages, then
#       opens the diagnostic GUI.
#
#  ASCII only on purpose: Windows PowerShell 5.1 misreads UTF-8 without BOM.
# =====================================================================
# Another folder: set DAIDALOS_TARGET first, e.g.  set DAIDALOS_TARGET=D:\Daidalos code
# (no param() block: this script is also run through "irm | iex").
$ErrorActionPreference = "Stop"
$Target = $env:DAIDALOS_TARGET

$RepoUrl    = "https://github.com/Tomasraketak/Daidalos_Landing_Rocket_Code.git"
$LandSimUrl = "https://github.com/Tomasraketak/LandSim_Light.git"
$AppDir     = Join-Path $env:LOCALAPPDATA "Daidalos"

function Say($msg) { Write-Host ">> $msg" -ForegroundColor Cyan }

function Refresh-Path {
    $m = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $u = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$m;$u"
}

function Have($cmd) { return [bool](Get-Command $cmd -ErrorAction SilentlyContinue) }

function Find-Python312 {
    # The py launcher knows every installed Python; ask it for 3.12.
    if (Have "py") {
        try {
            $p = (& py -3.12 -c "import sys; print(sys.executable)" 2>$null)
            if ($LASTEXITCODE -eq 0 -and $p -and (Test-Path $p)) { return $p.Trim() }
        } catch {}
    }
    $c = Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"
    if (Test-Path $c) { return $c }
    $c = Join-Path $env:ProgramFiles "Python312\python.exe"
    if (Test-Path $c) { return $c }
    return $null
}

# ---- target folder ------------------------------------------------------
if (-not $Target) {
    $od = Join-Path $env:USERPROFILE "OneDrive\Desktop"
    $desk = if (Test-Path $od) { $od } else { [Environment]::GetFolderPath("Desktop") }
    $Target = Join-Path $desk "Daidalos code"
}
$Parent  = Split-Path $Target -Parent
$LandSim = Join-Path $Parent "LandSim_Light"
New-Item -ItemType Directory -Force -Path $AppDir | Out-Null

# ---- tools ----------------------------------------------------------------
if (-not (Have "winget") -and (-not (Have "git") -or -not (Find-Python312))) {
    throw "winget neni k dispozici. Nainstalujte 'App Installer' z Microsoft Store, nebo rucne Git a Python 3.12."
}
if (-not (Have "git")) {
    Say "Instaluji Git..."
    winget install -e --id Git.Git --silent --accept-package-agreements --accept-source-agreements
    Refresh-Path
    if (-not (Have "git")) { $env:Path += ";$env:ProgramFiles\Git\cmd" }
}
if (-not (Find-Python312)) {
    Say "Instaluji Python 3.12..."
    winget install -e --id Python.Python.3.12 --scope user --silent --accept-package-agreements --accept-source-agreements
    Refresh-Path
}
$py = Find-Python312
if (-not $py) { throw "Python 3.12 se nepodarilo najit ani po instalaci." }
Say "Python: $py"

# ---- repositories ---------------------------------------------------------
function Ensure-Clone($url, $dir) {
    if (Test-Path (Join-Path $dir ".git")) { Say "$dir je uz git klon."; return }
    if ((Test-Path $dir) -and (Get-ChildItem -Force $dir | Select-Object -First 1)) {
        $bak = "$dir (stara kopie $(Get-Date -Format 'yyyyMMdd-HHmm'))"
        Say "Slozka $dir neni git klon - prejmenovavam ji na '$bak' (nic se nemaze)."
        Rename-Item -Path $dir -NewName (Split-Path $bak -Leaf)
    }
    Say "Stahuji $url -> $dir"
    git clone $url $dir
    if ($LASTEXITCODE -ne 0) { throw "git clone $url selhal." }
}
Ensure-Clone $RepoUrl $Target
Ensure-Clone $LandSimUrl $LandSim

# ---- config for the launcher ------------------------------------------------
@{ repo = $Target; landsim = $LandSim } | ConvertTo-Json | Set-Content -Encoding ASCII (Join-Path $AppDir "config.json")

# The launcher and the icon run from %LOCALAPPDATA%\Daidalos, not from the
# repository: git may rewrite the repository while the launcher is running,
# and OneDrive may keep a file "online only". The launcher copies its own
# newest version over after every update.
$win = Join-Path $Target "tools\windows"
Copy-Item -Force (Join-Path $win "daidalos_launch.ps1") (Join-Path $AppDir "daidalos_launch.ps1")
Copy-Item -Force (Join-Path $win "daidalos.ico")        (Join-Path $AppDir "daidalos.ico")

# ---- shortcut ---------------------------------------------------------------
$desktop = [Environment]::GetFolderPath("Desktop")
$lnk = Join-Path $desktop "Daidalos diagnostika.lnk"
$sh = New-Object -ComObject WScript.Shell
$s = $sh.CreateShortcut($lnk)
$s.TargetPath       = Join-Path $env:SystemRoot "System32\WindowsPowerShell\v1.0\powershell.exe"
$s.Arguments        = "-NoProfile -ExecutionPolicy Bypass -File `"$(Join-Path $AppDir 'daidalos_launch.ps1')`""
$s.WorkingDirectory = $Target
$s.IconLocation     = (Join-Path $AppDir "daidalos.ico") + ",0"
$s.Description      = "Daidalos: aktualizuje kod a Python, pak spusti diagnostiku"
$s.Save()
Say "Zastupce: $lnk"

Say "Prvni spusteni (vytvori prostredi Pythonu, to chvili trva)..."
& (Join-Path $AppDir "daidalos_launch.ps1")
