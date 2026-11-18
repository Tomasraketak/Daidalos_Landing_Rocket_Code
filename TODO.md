# Daidalos / Ariadna V2 — co ještě zbývá

Stav k firmwaru 3.20. ✅ = hotovo v kódu, ale **neověřeno na hardwaru**; ☐ = neudělané.
Odkazy: [`USER_MANUAL.md`](USER_MANUAL.md), [`CALIBRATION_MANUAL.md`](CALIBRATION_MANUAL.md).

## A. Software, který ještě chybí

| | úkol | proč | kde |
|---|---|---|---|
| ☐ | **Pozorovatel větru z LandSim_Light** (on-board wind observer) | LandSim (2. audit) odhaduje vítr ve volném pádu z gyra — bez něj počítají křidélka a dopředná kompenzace s rychlostí vůči zemi místo vůči vzduchu; v LandSim to odstranilo vzácné rozkmitání (1 let z 90 do 40–50° ještě před zážehem). V našem testu s větrem 1,8 m/s raketa přistane, ale silnější vítr testovaný není. | port `tvc_sim.py` (`WIND_OBS_*`, blok „on-board wind observer“) do `common/guidance.cpp`; přidat do `guid_test` případ s větrem 4–6 m/s |
| ✅ | **Zpoždění antény UWB** v kódu — konstanty `ANCHOR_ANTENNA_DELAY[]` / `TAG_ANTENNA_DELAY` (0 = výchozí); zbývá **změřit** (C) | firmware používá výchozí hodnotu knihovny arduino-dw1000; konstantní chyba vzdálenosti > 15 cm se dnes jen „schová“ do nejistoty | `Uwb_Anchor.ino` / `Uwb_Tag.ino`: konstanta `ANTENNA_DELAY` + `DW1000.setAntennaDelay()`; postup měření v `CALIBRATION_MANUAL.md` kap. 5.3 |
| ☐ | Steering zero-effort velocity (booster) | v LandSim jen s boosterem D9 — Daidalos ho nemá; **nepotřeba**, pokud se booster nepřidá | — |
| ☐ | Roztažení aerobrzdy do telemetrie / záznamu | firmware ho počítá (`brake_frac`), ale neposílá — po letu by šlo ověřit, jestli brzda opravdu byla venku | `ariadna_nav_status_t` + `LogRecord` |
| ☐ | Tabulka motoru a geometrie křidélek jako parametry | dnes zakompilované v `guidance_tables.h`; změna motoru = přegenerovat a nahrát firmware | `make tables VEHICLE=…` funguje; parametry by to zjednodušily |
| ☐ | `DG_CLAMP_MIN` (min. tah klapky) z LandSim | pevně 0,10 v `guidance.h`, generátor ho nepřepisuje | `tools/gen_guidance_tables.py` |
| ☐ | Import CSV z pozemní vizualizace do záložky Letový záznam | dnes umí jen formát záznamu z flash | `Diagnostics/diag_log.py` |
| ✅ | `tools/daidalos_log.py --flights-only` | GUI už stahuje jen lety, příkazová řádka zatím celé | `LOG FLIGHTS` + `LOG DUMP od počet` |
| ☐ | Celý odhadovač polohy do hlavičky a do `make check` | dnes testovaný po částech (filtr, UWB, trim), třída `Estimator` je v `.ino` | `Position_MCU.ino` → `common/` |

## B. Ověřit na hardwaru (kód je hotový)

| | co | jak |
|---|---|---|
| ☐ | **fw 3.21: link na stole** | GUI → řádek *M*: `pos`/`nav` = [dobré, vadné, živý]; vadné rámce mají růst jen výjimečně, `ok` trvale 1; *P* `hz` ≈ 200–240 (dřív 121); *N* `g` ≈ 1,00 v klidu a `p` drží ~0 m |
| ☐ | **fw 3.24: montáž TriSense (−Y = špička)** | záložka *Surová data IMU*: raketa svisle na rampě → akcelerometr ≈ (0, −1, 0) g; *Přehled*: 3-D raketa stojí, náklon < 2°; položit špičkou na sever / východ → směr náklonu 0° / 90°; obrátit špičkou dolů → „špička DOLŮ“, tah nahoru. Ověřit, že INSLIB se na rampě (roll desky ±90°) zarovná a nedrifuje |
| ☐ | **fw 3.25: kalibrace z GUI, rampa, kontrola serv** | po nahrání: parametry ze starého flash zůstaly (servo trimy); kalibrace akcelerometru a magnetometru v záložce Kalibrace a kontrola v *Surová data IMU*; „Rampa = tady“ s 3D fixem; kontrola serv: odpojit jedno servo → „NEREAGUJE“, připojená → „připojeno“; pokud šum proudu dělá falešné výsledky, upravit `mst.servo_check_ma` |
| ☐ | **fw 3.29: odhad baterie** | *Přehled → Master MCU*: „Nabití baterie“ a „Výdrž do limitu“. Plně nabitá 2S → ~100 % (fw 3.30: barvy oranžová < 60 min / červená < 20 min; po delším vybíjení `batt.learn` klesne, když baterie dává míň než `batt.health`, a po restartu drží)  a velká výdrž; po 10 s od zapnutí se objeví první odhad, pak se mění 1×/min; připojit stálou zátěž a ověřit, že výdrž odpovídá (podíl skutečné/ukázané → `batt.health`); v zimě ověřit teplotu (Position `temp`) a že výdrž klesá; zkontrolovat, že po SAVE/restartu `batt.*` drží a že starý flash (layout 5) se načte |
| ☐ | **fw 3.28: kanály a limity serv z GUI, mapa ČÚZK** | po nahrání: Master USB „servo map: TVC X 5 …“; *Aktuátory → Načíst z rakety* ukáže 5/6/7/8/9–12; změnit kanál nebo Max → *Zapsat do rakety* → restart → hodnoty drží; servo se za letového příkazu nedostane za Max; kontrola serv hlásí správná serva; pozemní stanice: MAP ⭳ PAD stáhne ortofoto, rampa „(rocket)“ sedí s místem rakety |
| ☐ | **fw 3.27: offset gyra do flash, serva 9–16, skupiny parametrů** | *Kalibrace → Kalibrovat gyro* → „Offset … uložen do flash ✓“ a v *Parametry → Kalibrace senzorů a rampa* jsou `imu.gyr_off_*`; restart s raketou v klidu → Position USB „agrees with the saved one“, restart při pohybu → „using the saved one“. *Aktuátory → Volné výstupy* (TEST): serva 9–16 se hýbou |
| ☐ | **fw 3.26: rychlost fúze** | GUI *Přehled → Position MCU → Fúze Hz* ≈ 230–240 (dřív 120); *Navigation → Smyčka max* a *Přetečení* Position bez zhoršení |
| ☐ | **Přeložit všechny sketche v Arduino IDE / arduino-cli** | testy na počítači používají náhradní hlavičky; arduino-pico, `--warnings all` |
| ☐ | **Letový záznam 200 Hz / 10 Hz** | na stole: ARMED → simulovat start (`mst.launch_g` / breakwire), 60 s; `LOG INFO`, Najít lety, stáhnout; v CSV kontrolovat rozestupy 5 ms a že `s_log_dropped` (diag *M* řádek) zůstal 0. Ověřit i při mazání sektorů (plný čip → prázdný) |
| ☐ | `LOG FLIGHTS` na plném čipu | prohledání 16 MB trvá ~5 s; během něj neběží LoRa (na zemi v pořádku) |
| ☐ | `LOG DUMP` rychlost | očekávat ~1000–2000 záznamů/s; let 30 s @ 200 Hz ≈ 6000 záznamů ≈ pár s |
| ☐ | Parametry na flash A/B | SAVE → vypnout → zapnout → „načteno z flash“; SAVE při vytažení napájení → po startu platná starší kopie |
| ☐ | Hradlo lidaru | nohy nevyklopené → „LIDAR povolen“ zhasnuté; PYRO3 → za `pos.lidar_delay_ms` se rozsvítí |
| ☐ | Montážní odchylka lidaru | `CALIBRATION_MANUAL.md` 4.2 |
| ☐ | **Znaménka TVC a křidélek** | `USER_MANUAL.md` kap. 12, kroky 3–4 — **nejdůležitější test celého projektu** |
| ☐ | Převod rámců INSLIB na rakety | self-test při startu + ruční test orientace (kap. 12 krok 5) |
| ☐ | INSLIB na reálných datech | jízda autem / let s GPS, porovnat s GPS |
| ☐ | DMA vysílání UART | ring je testovaný, konfigurace kanálu ne; záložně `ARIADNA_USE_DMA_TX 0` |
| ☐ | Ovladač UWB na desce | SPI/IRQ netestované na počítači |
| ☐ | Adaptace křidélek na skutečné aerodynamice | první let: `fin_gain` v záznamu |
| ☐ | Měření odporu v letu | první let: `drag` v záznamu ~1 (0,8–1,25) |
| ☐ | Dosah LoRa a pravidla pro 433 MHz | 10 Hz = 49 % vysílacího času (limit EU 10 %) — rozhodnout `ARIADNA_TLM_HZ` |

## C. Kalibrace (postup v `CALIBRATION_MANUAL.md`)

| | |
|---|---|
| ☐ | Akcelerometr (6 poloh) a magnetometr (MotionCal) → konstanty v `Position_MCU.ino` |
| ☐ | Deklinace místa startu |
| ☐ | Dělič baterie, ACS712, práh kontinuity roznětů; **model baterie** (`batt.health`, `batt.r25_mohm`) vybitím známým proudem |
| ☐ | Serva: střed, µs/°, **znaménka**, klapka, aerokryt, kontrola aerobrzdy 4 × 15° |
| ☐ | Lidar: offset, odchylka, zpoždění nohy, dosah na povrchu místa přistání |
| ☐ | UWB: zaměření kotev (před každým letem), zpoždění antén |
| ☐ | Hmotnost, těžiště, momenty setrvačnosti, geometrie křidélek |
| ☐ | Statický zážeh: tahová křivka (≥ 2 motory), zpoždění roznětu, klapka vs. tah, `g.onset_bump` |
| ☐ | LandSim: pásmo + ΔCd křidélek pro skutečnou raketu, při změně hmotnosti > 10 % i zisky |
| ☐ | Po prvním letu: Cd, škála tahu, účinnost křidélek z letového záznamu |

## D. Známá omezení (vědomě ponechaná)

* Apogeum nad ~190 m je mimo obálku samotného přistávacího motoru — přistání nejde uzavřít.
* Zisky se při změně hmotnosti nepřefitují samy (jen setrvačnost se škáluje) — `--gains`.
* Pod 2 m UWB nevidí výšku; spoléhá se na lidar a barometr kalibrovaný přes UWB.
* Mapování klapky µs → podíl tahu je lineární.
