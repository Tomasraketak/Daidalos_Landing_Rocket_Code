# Daidalos / Ariadna V2 — kalibrační manuál

Jak získat **každou** kalibrační hodnotu rakety: co se měří, čím, jak se z měření spočítá
číslo a kam se zapíše. Doplňuje [`USER_MANUAL.md`](USER_MANUAL.md), který popisuje, *jak*
algoritmy fungují. Tady je *čím je nakrmit*.

> ⚠️ Výchozí hodnoty v softwaru popisují **zamýšlenou** raketu ze simulace LandSim_Light,
> ne tu vaši. Než raketa poletí, každý řádek tabulky v kapitole 1 musí být buď změřený,
> nebo vědomě ponechaný na výchozí hodnotě, s poznámkou proč.

---

## Obsah

0. [Tři druhy kalibračních hodnot a kam se zapisují](#0-kam-se-co-zapisuje)
1. [Přehled — všechny hodnoty v jedné tabulce](#1-přehled)
2. [Nástroje](#2-nástroje)
3. [Senzory Position MCU (TriSense): gyro, akcelerometr, magnetometr, GPS](#3-senzory-position-mcu)
4. [Lidar na noze](#4-lidar-na-noze)
5. [UWB kotvy](#5-uwb-kotvy)
6. [Serva: TVC, klapka, křidélka, aerokryt](#6-serva)
7. [Elektrika: baterie, proud, kontinuita roznětů](#7-elektrika)
8. [Vozidlo pro LandSim: hmotnost, těžiště, setrvačnost, křidélka, Cd, aerobrzda](#8-vozidlo-pro-landsim)
9. [Motor: statický zážeh, tahová křivka, zpoždění roznětu, klapka](#9-motor)
10. [Po letu: co se dá změřit z letového záznamu](#10-po-letu)
11. [Pořadí kalibrace a záznamový list](#11-pořadí-a-záznamový-list)
12. [Když se raketa změní](#12-když-se-raketa-změní)

---

## 0. Kam se co zapisuje

| druh | kde žije | jak se změní | kdy se projeví |
|---|---|---|---|
| **parametr** (`g.*`, `srv.*`, `pos.*`, `seq.*`, `fin.*`, `mst.*`, `tbl.*`) | tabulka parametrů na flash čipu Mastera (W25Q128, sektory 0–1) | diagnostika → záložka **Parametry a časování** (nebo **Zážeh a LandSim**) → *Odeslat změny* → **Uložit do flash** | do pár sekund na všech deskách, bez přeprogramování |
| **konstanta v kódu** | `#define` / `static const` ve sketchi | upravit soubor, nahrát firmware dané desky | po nahrání |
| **vozidlo pro LandSim** | `vehicle.json` (záložka Zážeh a LandSim ho píše sama) | záložka **Zážeh a LandSim** → *Spočítat* → *Odeslat* → *Uložit do flash*; pro motor a geometrii navíc `make tables` | pásmo zážehu hned; tabulka motoru a geometrie po přeprogramování Navigation MCU |

Parametry se dají měnit **jen v IDLE nebo TEST** a jen na zemi. Po každé změně zkontrolujte
v horní liště diagnostiky **„Parametry: synchronní“** — teprve pak mají všechny desky stejnou
tabulku. Profil parametrů si po kalibraci **exportujte** (tlačítko v záložce Parametry): po
nahrání firmwaru s jiným rozložením tabulky (teď verze 3) se načtou výchozí hodnoty a profil
jde jen znovu importovat.

---

## 1. Přehled

Legenda: **P** = parametr, **K** = konstanta v kódu, **L** = vozidlo pro LandSim.
„Kdy“: **1×** = jednou pro desku/sestavu, **R** = po každé přestavbě rakety, **L** = před
každým letem, **Z** = zkontrolovat po letu.

| hodnota | druh | kde | čím se měří | kap. | kdy |
|---|---|---|---|---|---|
| bias gyra | — | automaticky při startu, tlačítko *Kalibrace gyra* | raketa v klidu 2 s | 3.1 | L |
| offset a měřítko akcelerometru `imu.acc_off_*`, `imu.acc_scale_*` | P | Position | diagnostika → Kalibrace (6+ poloh) | 3.2 | 1× |
| magnetometr `imu.mag_hi_*`, `imu.mag_si_*` | P | Position | diagnostika → Kalibrace (otáčení) | 3.3 | R |
| deklinace `MAG_DECLINATION_DEG` | K | `Position_MCU.ino` §2 | NOAA kalkulačka pro místo startu | 3.4 | podle místa |
| zpoždění GPS `pos.gps_latency_ms` | P | Position | datasheet modulu / měření | 3.5 | 1× |
| montáž TriSense, osa serva 1 | K | `BODY_SERVO1_AXIS` (Navigation) | kontrola, ne měření | 3.6 | R |
| výška lidaru nad zemí `pos.lidar_offset_m` | P | Position | metr | 4.1 | R |
| montážní odchylka lidaru `pos.lidar_tilt_x_deg`, `_y_deg` | P | Position | známá výška + náklon | 4.2 | R |
| zpoždění lidaru po vyklopení `pos.lidar_delay_ms` | P | Position | kamera 240 fps | 4.3 | R |
| dosah lidaru `pos.lidar_max_m` | P | Position | měření nad trávou | 4.4 | 1× |
| polohy UWB kotev `s_uwb_anchors[]` | K | `Position_MCU.ino` | pásmo, kompas | 5.1 | **L** |
| zpoždění antény UWB | K | `Uwb_Anchor` / `Uwb_Tag` | známá vzdálenost | 5.3 | 1× |
| střed TVC `srv.tvc_x_trim_us`, `_y_trim_us` | P | Master | úhloměr / laser | 6.2 | R |
| `srv.tvc_us_per_deg` | P | Master | úhloměr / laser | 6.3 | R |
| znaménka TVC `srv.tvc_x_sign`, `_y_sign` | P | Master | **statický test směru** | 6.4 | R |
| klapka `srv.clamp_closed_us`, `_open_us` | P | Master | poloha + statický zážeh | 6.5, 9.4 | R |
| střed křidélek `srv.fin1..4_trim_us` | P | Master | šablona / úhloměr | 6.6 | R |
| `srv.fin_us_per_deg` | P | Master | úhloměr | 6.6 | R |
| znaménka křidélek `srv.fin1..4_sign` | P | Master | **statický test směru** | 6.7 | R |
| aerokryt `srv.cover_lock_us`, `_unlock_us` | P | Master | ručně v TEST režimu | 6.8 | R |
| dělič baterie `VBAT_DIVIDER` | K | `Master_MCU.ino` | multimetr | 7.1 | 1× |
| proudové čidlo `ACS712_ZERO_V`, `ACS712_V_PER_A` | K | `Master_MCU.ino` | multimetr / známá zátěž | 7.2 | 1× |
| práh kontinuity `CONT_THRESHOLD`, `CONT_ACTIVE_HIGH` | K | `Navigation_MCU.ino` | ADC s roznětem / bez | 7.3 | 1× |
| hmotnost `g.mass_gross_kg` | P+L | Navigation | váha | 8.1 | **L** |
| palivo `g.mass_prop_kg` | P+L | Navigation | váha motoru plný/prázdný | 8.1 | R |
| průměr `g.diameter_m` | P+L | Navigation | posuvka | 8.2 | 1× |
| těžiště, rameno křidélek `fin_arm_m` | L | LandSim | vyvážení na hraně | 8.3 | R |
| momenty setrvačnosti `inertia` | L | LandSim | bifilární kyvadlo | 8.4 | R |
| plocha křidélka `fin_area_m2`, odtržení `fin_stall_deg` | L | LandSim | geometrie / CFD | 8.5 | R |
| Cd trupu `g.cd_body` | P+L | Navigation | odhad → **měří se v letu** | 8.6, 10.2 | Z |
| odpor křidélek a aerobrzdy `g.fin_dcd_zero`, `g.fin_dcd_brake` | P | Navigation | LandSim (CFD) | 8.7 | R |
| tahová křivka motoru | L (+ K) | LandSim, `guidance_tables.h` | tenzometr, statický zážeh | 9.1 | každá šarže motorů |
| škála tahu `g.thrust_scale` | P+L | Navigation | impulz ze statického zážehu | 9.2 | Z |
| zpoždění roznětu `ign_delay_max_s`, `g.ign_delay_pad_s` | L+P | LandSim → Navigation | statický zážeh | 9.3 | R |
| práh detekce tahu `g.onset_bump` | P | Navigation | akcelerometr při statickém zážehu | 9.5 | 1× |
| práh startu `mst.launch_g` | P | Master | hlavní motor (datasheet) | 9.6 | R |
| pásmo zážehu `tbl.*` | P | Navigation | **počítá LandSim** z hodnot výše | 8.8 | po každé změně L |
| zisky regulátorů `g.wn_*`, `g.zeta_*`, `g.sched_*`, `g.roll_gain`, `g.att_ki` | P | Navigation | **LandSim `--gains`** | 12 | při změně hmotnosti > ~10 % |
| výchozí účinnost křidélek `fin.gain0` | P | Navigation | z letového záznamu | 10.3 | Z |

---

## 2. Nástroje

**Měřidla:** kuchyňská váha (0,1 g) a závěsná váha do 5 kg, posuvné měřítko, svinovací metr
a 30m pásmo, digitální úhloměr nebo mobil s aplikací inklinometru (0,1°), laserové ukazovátko
přilepené na trysku (pro TVC), multimetr, zdroj s omezením proudu, dva provázky 1–2 m
(bifilární kyvadlo), stopky nebo kamera 240 fps (mobil ve zpomaleném režimu),
**tenzometr se zesilovačem** (HX711 nebo lepší, ≥ 1 kHz) pro statický zážeh.

**Software:**

| nástroj | kde | k čemu |
|---|---|---|
| diagnostická konzole | `Ariadna_V2/Diagnostics/daidalos_diag.py` | parametry, TEST režim, serva, ADC, senzory |
| LandSim_Light | `git clone https://github.com/Tomasraketak/LandSim_Light` | pásmo zážehu, odpor křidélek, zisky |
| *IMUcalibration*, *MotionCal* | příklady knihovny Voltino_TriSense | akcelerometr, magnetometr |
| PJRC MotionCal | https://www.pjrc.com/store/prop_shield.html | fit magnetometru |
| stažení letového záznamu | `tools/daidalos_log.py` | CSV z flash Mastera (kap. 10) |
| bench testy | `Ariadna_V2/Bench_Tests/` | kontrola senzorů a spojů po desce |

Příklady knihovny se nahrávají **místo** letového firmwaru Position MCU. Po kalibraci vraťte
letový firmware.

---

## 3. Senzory Position MCU

### 3.1 Gyroskop — nic se nezapisuje

Bias gyra se měří **při každém startu** (raketa musí 2 s stát v klidu) a znovu tlačítkem
**Kalibrace gyra** v diagnostice (příkaz `CAL GYRO`, jen v IDLE/TEST). Za letu se dál
dolaďuje (INSLIB odhaduje bias gyra jako stav filtru).

**Kdy:** na rampě, po usazení rakety a před zastrčením. Raketou se během kalibrace nesmí
hýbat — ani větrem. Kontrola: v záložce *Poloha a senzory* úhlové rychlosti v klidu
< 0,1 °/s a náklon se za minutu nezmění o víc než 0,2°.

### 3.2 Akcelerometr — 6 poloh (jednou pro každý TriSense)

Akcelerometr má offset a chybu měřítka. Filtr si bias akcelerometru odhaduje sám, ale
měřítko ne, a na rampě se z akcelerometru počítá počáteční náklon.

Od fw 3.25 **v diagnostice, záložka Kalibrace** — nic se nepřepisuje do kódu:

1. Deskou/raketou položte v klidu v nějaké poloze a stiskněte **Zachytit polohu** (2 s
   průměr surových dat; při pohybu se zachycení odmítne).
2. Opakujte pro další polohy. Stačí **6 libovolných, rozmanitých** poloh; nejlepší je 6 stran
   (každá osa jednou nahoru a jednou dolů) a 7. a další poloha fit ověří (se 6 polohami vyjde
   zbytková chyba vždy 0). Skoro stejné polohy (< 20°) GUI odmítne.
3. GUI průběžně ukazuje offset, škálu a zbytkovou chybu |a| − 1 (má být < 3 mg).
4. **Použít a uložit do flash** → parametry `imu.acc_off_x/y/z`, `imu.acc_scale_x/y/z`
   (`a = (raw − off) · scale`, osy desky) se zapíšou do flash Masteru a Position MCU je
   použije okamžitě.

(Starý postup s příkladem `IMUcalibration` z knihovny Voltino_TriSense dává stejná čísla;
zapisují se do stejných parametrů.)

**Kontrola:** raketa svisle na rampě → v diagnostice |a| = 1,000 ± 0,005 g, náklon
odpovídá vodováze ± 0,3°. Typické hodnoty: offset do ±0,05 g, měřítko 0,98–1,02.

### 3.3 Magnetometr — hard/soft iron (po každé přestavbě elektroniky)

Magnetometr určuje směr k severu na rampě (azimut), dokud GPS nedá rychlost. Kovy a proudy
v raketě (baterie, serva, motor!) ho zkreslují, proto se kalibruje **v sestavené raketě**,
s baterií na místě a vypnutými servy.

Od fw 3.25 **v diagnostice, záložka Kalibrace → Magnetometr**:

1. **Start sběru** (samo zapne surová data).
2. Otáčejte celou raketou pomalu všemi směry (osmičky, převracení přes všechny strany),
   asi minutu, až **pokrytí směrů ≥ 85 %**.
3. **Stop a spočítat**: GUI proloží obecný elipsoid surovými vzorky a ukáže hard iron (µT),
   matici soft iron, sílu pole (u nás ~48 µT) a rozptyl |B| před/po (po má být < 2–3 %).
4. **Použít a uložit do flash** → `imu.mag_hi_x/y/z` a `imu.mag_si_00…22`
   (`m = SI · (raw − HI)`, konvence MotionCal, kterou používá INSLIB).

**Kontrola:** raketa na rampě, otočit o 90° → azimut v diagnostice se změní o 90 ± 5°.
Kalibraci opakujte po přesunu baterie, serv nebo kabeláže.

### 3.4 Magnetická deklinace

`MAG_DECLINATION_DEG` v `Position_MCU.ino` (výchozí 5,6° = střední Čechy 2026). Hodnotu pro
místo startu zjistíte na https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
(kladná = východní). Po prvním GPS fixu ji INSLIB nahradí modelem WMM, takže je důležitá
jen bez GPS.

### 3.5 Zpoždění GPS `pos.gps_latency_ms`

Jak staré je GPS měření, když dorazí. Filtr ho porovnává s polohou, kde raketa *byla*.

* **Z datasheetu:** většina modulů uvádí „navigation latency“ 20–80 ms. Výchozí 50 ms.
* **Měřením (volitelně):** raketu (nebo desku s GPS) nést rychlou chůzí po přímce,
  prudce zastavit, a v CSV záznamu z diagnostiky (*Nahrávat*) porovnat okamžik zastavení
  podle akcelerometru a podle GPS rychlosti. Rozdíl je latence. Měřte 5× a vezměte průměr.

Chyba 20 ms při 30 m/s je 0,6 m — filtr ji unese, jen pomaleji věří GPS.

### 3.6 Montáž TriSense a osa serva 1 — kontrola, ne měření

Tělesové osy rakety **jsou** osy TriSense. Daidalos (fw 3.24): **−Y míří ke špičce**
(na rampě nahoru; po překlopení v apogeu je při přistání nahoře +Y), osa tahu
přistávacího motoru je +Y, +X je osa, na které leží servo TVC 1 a křidélko 1
(`BODY_SERVO1_AXIS = {1,0,0}` v `Navigation_MCU.ino`), +Z je druhá radiální osa. Montáž je
zapsaná na jediném místě: `BODY_NOSE_AXIS` / `BODY_THRUST_AXIS` v `Position_MCU.ino`
(a `BODY_NOSE` / `BODY_FIN1` v `Diagnostics/daidalos_widgets.py` pro GUI). Pokud je deska
otočená jinak, změňte tyto konstanty nebo `BODY_SERVO1_AXIS` — a pak **znovu udělejte testy
znamének** (kap. 6.4 a 6.7). Rychlá kontrola: záložka *Surová data IMU* — raketa svisle
na rampě ukazuje akcelerometr ≈ (0, −1, 0) g. Kontrola orientace: `USER_MANUAL.md`
kap. 12, krok 5.

---

## 4. Lidar na noze

Lidar TF-Luna je na vyklápěcí noze. Do výšky se započte až `pos.lidar_delay_ms` po
přepálení gumičky nohou, jen když jeho hodnota není na stropu rozsahu, a s korekcí na náklon
rakety a na montážní odchylku (viz `USER_MANUAL.md` kap. 4.4).

### 4.1 Offset `pos.lidar_offset_m`

Výška, kterou má raketa ukazovat, = vzdálenost lidaru × cos(úhel paprsku) +
`pos.lidar_offset_m`. Výška 0 je **dosednutí** (rampa = výška nul, kde stojí nohy).

1. Raketu postavte na nohy (vyklopené), svisle, na rovnou zem.
2. V záložce *Poloha a senzory* přečtěte vzdálenost lidaru `d`.
3. `pos.lidar_offset_m = −d` (lidar je nad zemí, takže offset je záporný; typicky −0,2 až
   −0,5 m).

Ověření: zvedněte raketu (svisle) o změřených 1,00 m → výška lidaru 1,00 ± 0,03 m.

### 4.2 Montážní odchylka `pos.lidar_tilt_x_deg`, `pos.lidar_tilt_y_deg`

Úhel, o který paprsek lidaru míří vedle podélné osy rakety, když je noha vyklopená
(noha nikdy nedosedne přesně). Paprsek bez odchylky míří podél špičky (tělesová −Y).
Kladná `tilt_x` = otočení kolem tělesové +X (paprsek se odkloní k −Z), kladná `tilt_y` =
otočení kolem +Z (paprsek k +X). (Do fw 3.23 byla druhá osa +Y — staré hodnoty `tilt_y`
přeměřte.)

**Metoda A — přímo (nejrychlejší):** raketu zavěste špičkou dolů (nebo položte na bok
s lidarem proti stěně) tak, aby podélná osa byla přesně kolmo na stěnu ve vzdálenosti
2–3 m. Na stěně označte, kam míří osa rakety (olovnice / laser podél trupu), a kam svítí
lidar (TF-Luna svítí v blízké IR — **mobilní kamerou** je bod vidět). Odchylka bodu od
osy `e_x`, `e_y` ve vzdálenosti `L`:

```
tilt = atan(e / L)        (e ve směru tělesové −Z → kladná tilt_x;  ve směru +X → kladná tilt_y)
```

**Metoda B — z výšky:** raketa svisle (špičkou dolů) ve známé výšce `H` nad rovnou zemí,
noha vyklopená. Naklánějte raketu kolem osy X o −10, 0, +10°. Výška z lidaru se mění;
`tilt_x` je úhel, při kterém je *naměřená vzdálenost nejkratší*, s opačným znaménkem.
Totéž kolem Z (`tilt_y`). V diagnostice nastavte nalezené hodnoty a ověřte, že výška z lidaru je při
±10° náklonu stejná (± 2 cm).

### 4.3 Zpoždění po vyklopení `pos.lidar_delay_ms`

Doba od příkazu PYRO3 (přepálení gumičky) do chvíle, kdy noha **stojí v dorazu a nekmitá**.

1. TEST režim, záložka Aktuátory → PYRO3 s gumičkou a nohou (bez motoru!).
2. Natáčejte kamerou 240 fps, v záběru LED pyro kanálu nebo kontrolka zdroje.
3. Spočítejte snímky od rozsvícení do ustálení nohy: `t = snímky / 240`.
4. `pos.lidar_delay_ms = t + 100 ms rezerva`. Výchozí 500 ms.

Za letu proudí vzduch — noha se může otevírat pomaleji. Kontrola v letovém záznamu: událost
**Lidar** se má rozsvítit nejpozději 0,3 s po **Nohy**.

### 4.4 Dosah `pos.lidar_max_m`

TF-Luna dosahuje 8 m na bílé ploše, na trávě a hlíně méně. Nad místem přistání (stejný
povrch!) zvedejte raketu a sledujte, do jaké výšky měření nekolísá víc než ±5 cm a síla
signálu je v normě. `pos.lidar_max_m` = tato výška − 0,5 m (výchozí 7,5 m). Strop 7,8 m
(„vymaxovaná“ hodnota) je pevně v kódu (`LIDAR_CEILING_M`).

---

## 5. UWB kotvy

### 5.1 Poloha kotev — před každým letem

`s_uwb_anchors[]` v `Position_MCU.ino`: adresa, x (východ), y (sever), z (nahoru) v metrech
**vůči rampě** (střed rampy = 0, 0, 0; z = výška antény kotvy nad úrovní rampy).

1. Kotvy rozmístěte do čtverce kolem rampy (výchozí ±20 m). Čtverec je nejlepší geometrie,
   **nikdy ne do řady**.
2. Směr na sever určete kompasem (opravit o deklinaci!) nebo dvěma GPS body.
3. Pásmem změřte každou kotvu: vzdálenost od rampy a úhel, nebo lépe pravoúhle — vzdálenost
   podél osy východ a sever. Přesnost ±5 cm. Změřte i diagonály čtverce jako kontrolu
   (`d = √(Δx² + Δy²)`).
4. Výšku antény každé kotvy nad úrovní rampy (svahovaný terén!) změřte vodováhou s latí nebo
   nivelačním přístrojem.
5. Zapište do `s_uwb_anchors[]` a nahrajte firmware Position MCU. **Číslo kotvy (ANCHOR_ID)
   napište na krabičku** — prohozené kotvy dají sebevědomě špatnou polohu.

**Kontrola na místě:** raketa na rampě → v diagnostice poloha (0, 0) ± 0,2 m a **post-fit
reziduál < 20 cm**. Raketu přeneste na změřený bod (např. 5 m na východ) → poloha souhlasí.
Reziduál, který neklesne pod ~30 cm, je chyba zaměření, ne rádia. LED Position MCU „žlutě 3×“
= baro trim na mezi = kotvy zaměřené vedle.

### 5.2 Kotva navíc

Tabulka má 8 míst. Kotva na 3m tyči u rampy výrazně zlepší výšku z UWB v posledních metrech
(viz `Ariadna_V2/README.md`, UWB). Zapsat stejně, `z` = výška antény.

### 5.3 Zpoždění antény (jednou pro každou kotvu a tag)

Každý DWM1000 má vlastní zpoždění antény, které se projeví jako konstantní přičtená
vzdálenost.

1. Tag a jednu kotvu postavte na stativy přesně 5,00 m od sebe (anténa–anténa), volný prostor.
2. V záložce *Poloha a senzory* odečtěte vzdálenost k té kotvě (průměr ze 100 měření).
3. Chyba `e = změřeno − 5,00 m`. Do ±0,15 m nic nedělejte (filtr má podlahu nejistoty
   pro tuto chybu).
4. Jinak se chyba rozdělí mezi oba konce: k zpoždění kotvy i tagu přičtěte
   `e / 0,00469 / 2` jednotek (výchozí hodnota knihovny je 16384):
   * kotva: `ANCHOR_ANTENNA_DELAY[číslo − 1]` v `Uwb_Anchor.ino`,
   * tag: `TAG_ANTENNA_DELAY` v `Uwb_Tag.ino` (0 = výchozí).
5. Nahrát firmware, změřit znovu; pak stejně pro ostatní kotvy (už jen kotvu, tag je hotový).

---

## 6. Serva

Všechno v **TEST režimu** (tlačítko *TEST režim*), záložka **Aktuátory a pyro**: posuvníky
nastavují pulz v µs, nalezené hodnoty se pak zapíšou do záložky **Parametry a časování**
→ *Odeslat* → *Uložit do flash*. **Jen digitální serva** (PWM 339 Hz).

### 6.1 Než začnete

Nastavte bezpečné meze serv v konzoli (*konfigurace serv*), aby servo nenajelo do dorazu
mechaniky. Pyro kanály odpojené.

### 6.2 Střed trysky `srv.tvc_x_trim_us`, `srv.tvc_y_trim_us`

1. Na trysku přilepte laserové ukazovátko podél její osy, raketu upněte svisle, 2–3 m pod
   trysku (nebo na strop) dejte papír.
2. Označte bod, kam svítí osa trupu (laser na trupu nebo olovnice z osy).
3. Posouvejte servo X, dokud laser trysky není na čáře osy v X; totéž Y. Opište pulzy.

### 6.3 Strmost TVC `srv.tvc_us_per_deg`

Navedení posílá **úhel serva**, a tryska se vychýlí o polovinu (`DG_TVC_RATIO = 2`).
Strmost proto měříte na **trysce** a přepočítáte:

1. Ze středu posuňte servo X o +100 µs, změřte posun laseru `s` na papíře ve vzdálenosti `L`.
2. Úhel trysky `θ = atan(s / L)` ve stupních.
3. `srv.tvc_us_per_deg = 100 / (2 · θ)`. Totéž −100 µs a osa Y; vezměte průměr (osy by se
   neměly lišit o víc než 5 %, jinak jsou páky různé — srovnejte mechaniku).

Kontrola: celý rozsah serva ±10° musí dát trysku ±5° bez dotyku.

### 6.4 Znaménka TVC — NEJDŮLEŽITĚJŠÍ TEST

Postup je v `USER_MANUAL.md` kap. 12, krok 3. Shrnutí: kladný příkaz osy 1 vychýlí **tah**
k tělesové +X, tedy **výtokový proud** trysky k −X. Pokud ne, `srv.tvc_x_sign = −1`. Osa 2:
výtok k +Y. **Obě osy, vždy.** Špatné znaménko = raketa se do 1 s převrátí.

### 6.5 Klapka (clamp) `srv.clamp_closed_us`, `srv.clamp_open_us`

Klapka odklání část proudu: `k = 1` plný tah, `k = 0,10` minimum (pevně v kódu
`DG_CLAMP_MIN`, z LandSim). Firmware mapuje `k` na pulz **lineárně** mezi `closed` a `open`.

1. `srv.clamp_open_us`: poloha, kdy klapka vůbec nezasahuje do proudu.
2. `srv.clamp_closed_us`: poloha největšího odklonu (mechanický doraz − 20 µs).
3. Že `closed` opravdu dává ~10 % tahu a že je závislost lineární, ověřuje statický zážeh
   (kap. 9.4).

### 6.6 Křidélka: střed a strmost `srv.finN_trim_us`, `srv.fin_us_per_deg`

Křidélka dělají při pádu **aerobrzdu**: roztáhnou se střídavě +,−,+,− (až `fin.max_deg` =
15°). Brzda je bez momentu jen tehdy, když jsou **všechna čtyři ve stejném úhlu** — nepřesný
střed jednoho křidélka znamená, že brzda raketu roztáčí. Proto střed nastavujte pečlivě:

1. Vytiskněte šablonu 0° (nebo použijte úhloměr přiložený k trupu) a posouvejte každé
   křidélko, dokud jeho tětiva není rovnoběžná s osou rakety. Přesnost lepší než 0,3°.
2. Strmost: křidélko 1 posuňte o +150 µs, změřte úhel `α` (°). `srv.fin_us_per_deg =
   150 / α`. Ověřte i na ostatních (společná hodnota pro všechna čtyři — liší-li se o víc než
   5 %, srovnejte páky).
3. **Kontrola aerobrzdy:** v TEST režimu nastavte křidélka na `trim ± 15° × us_per_deg`
   střídavě (+ na 1 a 3, − na 2 a 4, se svými znaménky). Úhly změřte úhloměrem: všechny
   čtyři 15 ± 0,5°. Toto je přesně poloha, ve které raketa padá.

### 6.7 Znaménka křidélek `srv.finN_sign`

Postup a tabulka směrů: `USER_MANUAL.md` kap. 12, krok 4. Kladná výchylka musí dát sílu ve
směru `b × r_i`. Kontrola navíc: všechna čtyři stejně kladně → moment kolem podélné osy;
tlumič rotace ho pak musí tlumit, ne budit.

### 6.8 Aerokryt `srv.cover_lock_us`, `srv.cover_unlock_us`

V TEST režimu tlačítky *Zajistit/Odjistit* a posuvníkem najděte polohy, kdy je kryt jistě
zajištěný (s rezervou proti vibracím) a jistě uvolněný. *Uložit polohy* je uloží do
konfigurace konzole (JSON, pro ruční testy); **pro let** je přepište do parametrů
`srv.cover_lock_us` / `srv.cover_unlock_us` (záložka Parametry → Odeslat → Uložit do flash).
Kontrola: 10× za sebou odjistit pod zátěží pružiny/krytu — pokaždé se uvolní.

---

## 7. Elektrika

### 7.1 Napětí baterie `VBAT_DIVIDER` (`Master_MCU.ino`)

1. Změřte multimetrem napětí baterie `U_m` a přečtěte v diagnostice `U_d` (horní lišta).
2. `VBAT_DIVIDER_nový = VBAT_DIVIDER_starý × U_m / U_d`. Výchozí 3,00 (20 k / 10 k).
3. Ověřte při dvou napětích (nabitá a vybitá baterie): chyba < 0,05 V.

### 7.2 Proud `ACS712_ZERO_V`, `ACS712_V_PER_A` (`Master_MCU.ino`)

`I = (U_pin / ACS712_DIVIDER − ACS712_ZERO_V) / ACS712_V_PER_A`.

1. **Nula:** bez zátěže (serva vypnutá) přečtěte proud `I_0` v diagnostice.
   `ACS712_ZERO_V_nový = ACS712_ZERO_V + I_0 × ACS712_V_PER_A`.
2. **Strmost:** zátěž se známým proudem `I_s` (např. výkonový rezistor, proud ověřený
   multimetrem, 2–5 A). Diagnostika ukazuje `I_d`.
   `ACS712_V_PER_A_nový = ACS712_V_PER_A × I_d / I_s`.

### 7.3 Kontinuita roznětů `CONT_THRESHOLD`, `CONT_ACTIVE_HIGH` (`Navigation_MCU.ino`)

1. TEST režim, záložka Aktuátory → pyro panel ukazuje **surové ADC** každého kanálu.
2. Do kanálu zapojte skutečný roznět (stejný typ jako v letu) → `A_ok`. Bez roznětu → `A_open`.
3. `CONT_THRESHOLD = (A_ok + A_open) / 2`. Pokud `A_ok > A_open`, `CONT_ACTIVE_HIGH = 1`,
   jinak 0. Rozdíl by měl být aspoň 500 (z 4095), jinak je měřicí obvod špatně navržený.
4. Ověřte na všech třech kanálech a s roznětem na dlouhém kabelu, jaký bude v raketě.

---

## 8. Vozidlo pro LandSim

Tyto hodnoty jdou do záložky **Zážeh a LandSim** a z ní do LandSim_Light. Vlevo jsou
parametry, které čte i raketa (`g.*`), vpravo (karta LandSim) geometrie, kterou potřebuje
jen simulace. Prázdné pole („výchozí LandSim“) = hodnota referenční rakety.

### 8.1 Hmotnost `g.mass_gross_kg`, palivo `g.mass_prop_kg`

* `g.mass_gross_kg`: **celá raketa připravená ke startu**, s přistávacím motorem (plným),
  bez hlavního motoru, pokud ho raketa v apogeu nenese. Pokud nese (prázdný) obal hlavního
  motoru, zvažte ho s ním. Váha ±5 g. **Před každým letem** — každý přidaný gram posouvá
  pásmo zážehu.
* `g.mass_prop_kg`: hmotnost paliva přistávacího motoru = plný motor − vyhořelý motor (z
  datasheetu nebo zvážením motoru po statickém zážehu).

### 8.2 Průměr `g.diameter_m`

Posuvkou největší průměr trupu (bez křidélek). Z něj se počítá referenční plocha pro všechna
Cd.

### 8.3 Těžiště a rameno křidélek `fin_arm_m`

1. Raketu připravenou k letu (v konfiguraci při **pádu**: hlavní motor vyhořelý/odhozený)
   položte na hranu (pravítko na stole) a hledejte bod rovnováhy. Změřte jeho vzdálenost od
   špičky.
2. `fin_arm_m` = vzdálenost těžiště → střed tlaku křidélka. Střed tlaku je přibližně ve
   **čtvrtině střední aerodynamické tětivy** od náběžné hrany. Pro lichoběžníkové křidélko s
   kořenovou tětivou `c_r` a koncovou `c_t`:
   `c_mac = 2/3 · (c_r + c_t − c_r·c_t / (c_r + c_t))`.
3. Na rameni závisí účinnost křidélek; adaptace za letu (`fin.adapt`) opraví ±50 %, ale
   pásmo a zisky počítá LandSim s touto hodnotou.

### 8.4 Momenty setrvačnosti `inertia.transverse`, `inertia.roll` (bifilární kyvadlo)

Hodnoty v kg·m² **při plné hmotnosti** (`g.mass_gross_kg`).

**Příčný moment (kolem osy kolmé na trup, přes těžiště):**
raketu zavěste vodorovně na dva svislé provázky délky `L`, upevněné ve vzdálenosti `d/2`
na obě strany od těžiště (symetricky). Rozkývejte ji o malý úhel (< 10°) **kolem svislé osy**
přes těžiště a změřte dobu `T` 20 kmitů / 20.

```
I = m · g · d² · T² / (16 · π² · L)
```

**Podélný moment (roll):** raketu zavěste svisle na dva provázky upevněné na obvodu trupu
(vzdálenost `d` = průměr), stejný vzorec. Roll je malý (~0,004 kg·m²), proto použijte delší
provázky a měřte 50 kmitů.

Kontrola řádu: tenký válec `I_t ≈ m·l²/12`, `I_r ≈ m·r²/2`. LandSim bez zadání použije
přesně tyto odhady.

### 8.5 Křidélka: plocha `fin_area_m2` a odtržení `fin_stall_deg`

* `fin_area_m2`: plocha **jednoho** křidélka (viditelná část mimo trup). Lichoběžník:
  `(c_r + c_t) / 2 · rozpětí`.
* `fin_stall_deg`: úhel náběhu, při kterém se proudění odtrhne. CFD v LandSim končí na
  10°; výchozí 22° je odhad pro křidélka s malou štíhlostí. Pokud máte jiný tvar
  křidélek, nechte výchozí, ale **nezvyšujte `fin.max_deg` nad 15°**.
* Pokud zadáte vlastní plochu nebo `fin_cl_alpha`, LandSim přejde z CFD tabulky na
  analytický model — odpor křidélek (kap. 8.7) se pak spočítá z něj.

### 8.6 Cd trupu `g.cd_body` — odhad, pak měření v letu

Cd předem přesně nevíte a **nemusíte**: raketa si odpor ve volném pádu měří sama
(`g.drag_adapt = 1`, poměr „Odpor měřený / model“ v diagnostice a v letovém záznamu).
Přesto začněte rozumným odhadem, protože z něj LandSim počítá pásmo a z něj se odvozuje
výška zážehu před tím, než měření doběhne:

1. **OpenRocket / RockSim:** namodelujte trup *bez křidélek*, odečtěte Cd při Machu 0,1
   **pro let zádí napřed** (raketa padá motorem dolů = obráceně! zadní část je teď vpředu —
   v OpenRocketu otočte model). Typicky 0,3–0,6.
2. Bez modelu: výchozí 0,35 je referenční raketa.
3. **Po prvním letu** opravte z letového záznamu (kap. 10.2):
   `g.cd_body_nový ≈ (g.cd_body + g.fin_dcd_brake) × drag − g.fin_dcd_brake`.

### 8.7 Odpor křidélek a aerobrzdy `g.fin_dcd_zero`, `g.fin_dcd_brake`

Při volném pádu jsou křidélka roztažená jako **aerobrzda** (+,−,+,−, 15°): asi zdvojnásobí
odpor a zpomalí pád, což zmenší potřebný impulz motoru. Při zážehu se zasunou (neutrál).
Raketa proto potřebuje dvě čísla, obě jako **přírůstek k Cd trupu** (na ploše trupu):

| parametr | kdy platí | výchozí |
|---|---|---|
| `g.fin_dcd_zero` | křidélka v neutrálu (hoření) | 0,143 |
| `g.fin_dcd_brake` | křidélka roztažená jako brzda (pád) | 0,447 |

**Nezadávají se ručně** — spočítá je LandSim (CFD tabulka křidélek nebo analytický model z
kap. 8.5) a záložka *Zážeh a LandSim* je po výpočtu sama vyplní (`band.json`:
`fin_cd_zero − cd_body`, `fin_cd_brake − cd_body`).

Jak s nimi raketa zachází:

* Brzda má přednost **až po řízení**: když regulátor potřebuje výchylku, brzda se zmenší.
  Firmware počítá, kolik brzdy skutečně dostal (střídavý součet čtyř výchylek), a podle
  toho míchá Cd mezi `zero` a `brake` (∝ druhé mocnině roztažení). Měřený odpor tak
  porovnává s brzdou, jaká opravdu je.
* `fin.brake = 0` brzdu vypne (pak se celý pád počítá s `g.fin_dcd_zero`) — a **pásmo musí
  být přepočítané** s vypnutou brzdou (pole „Křidélka jako brzda“ v záložce Zážeh).
* Ověření brzdy na zemi: kap. 6.6, bod 3. Ověření v letu: „Odpor měřený / model“ blízko 1
  (0,8–1,25); hodně pod 1 → brzda se neotevírá naplno (slabá serva, vůle).

### 8.8 Výpočet pásma zážehu (po každé změně čehokoli v kap. 8 a 9)

1. Záložka **Zážeh a LandSim**: vyplnit vozidlo, cestu ke klonu LandSim_Light.
2. **Spočítat v LandSim_Light** (`landsim_export.py`, ~15–60 s). Diagnostika sama posílá
   `n_boosters: 0` — Daidalos nemá D9 booster, výchozí raketa LandSim ano.
3. Zkontrolovat graf a náhled: výška zážehu pro vaše apogeum, obálka (červeně = mimo).
4. **Odeslat vozidlo + tabulku do rakety** → **Uložit do flash**.

---

## 9. Motor

### 9.1 Tahová křivka (statický zážeh na tenzometru)

1. Motor ve stojanu na tenzometru (≥ 1 kHz vzorkování, kalibrace závažím před i po).
2. Zaznamenat průběh, odečíst nulu, převést na newtony.
3. Uložit jako CSV `t,N` (čas od prvního nárůstu tahu nad 2 % špičky, ~20–40 bodů stačí,
   konec v nule).
4. Záložka Zážeh a LandSim → **Tahová křivka (CSV t,N)** → Spočítat. Pásmo pak odpovídá
   vašemu motoru.
5. ⚠️ Tabulka tahu, ze které raketa **za letu** plánuje, je zakompilovaná
   (`guidance_tables.h`, 17 bodů). Pro jinou křivku ji přegenerujte:
   *Uložit vehicle.json…* a pak
   ```bash
   cd test && make tables SIM=../../LandSim_Light VEHICLE=../vehicle.json
   ```
   (přegeneruje motor, geometrii i pásmo; pak `make check` — test parametrů upozorní, které
   výchozí hodnoty v `ariadna_params.h` srovnat) a nahrajte firmware Navigation MCU.
   Pro stejný typ motoru jen s jiným výkonem stačí `g.thrust_scale` (kap. 9.2).

Změřte **aspoň 2 motory** stejné šarže — rozptyl mezi kusy je pro navedení důležitější než
střední hodnota.

### 9.2 Škála tahu `g.thrust_scale`

`g.thrust_scale = celkový impulz změřený / celkový impulz tabulky` (tabulka = křivka v
`guidance_tables.h`, motor „long“ z LandSim). Při 2 a více statických zážezích průměr.
Za letu raketa tah měří akcelerometrem (`Tah / tabulka` v diagnostice, `tscale` v záznamu)
a plánovač ho používá; `g.thrust_scale` je jen výchozí odhad pro výpočet pásma a pro
okamžik zážehu.

### 9.3 Zpoždění roznětu: `ign_delay_max_s` (LandSim) a `g.ign_delay_pad_s`

Doba od sepnutí PYRO1 do nárůstu tahu nad ~5 % špičky. Pásmo zážehu je pro **začátek
tahu**, proto se příkaz posílá dřív o rezervu `g.ign_delay_pad_s`.

1. Při statickém zážehu zaznamenejte současně sepnutí pyro výstupu (druhý kanál
   osciloskopu / ADC, nebo LED v záběru kamery 240 fps) a tenzometr.
2. Změřte 3–5 roznětů stejného typu a stejné přípravy (namáčení, tlak, teplota!).
3. `ign_delay_max_s` (karta LandSim) = nejdelší naměřené zpoždění + 50 ms.
4. Pole `g.ign_delay_pad_s` vyplní LandSim (optimum pro dané max. zpoždění); ručně jen pokud
   víte proč. Zapálit výš se dá opravit škrcením, zapálit níž ne.
5. `seq.backup_win_s` (okno záložního roznětu, 150 ms) má být **větší** než typické
   zpoždění a menší než `ign_delay_max_s` + 0,1 s.

### 9.4 Klapka vs. tah

Při statickém zážehu s klapkou: v TEST režimu nastavte klapku na pevnou polohu (zážeh je
krátký — jedna poloha na motor), změřte poměr tahu proti zážehu s klapkou otevřenou.

* `srv.clamp_closed_us` má dát ~10 % tahu (`DG_CLAMP_MIN = 0,10`). Pokud dává výrazně víc,
  nastavte LandSim `k_min` (vehicle.json, např. `"k_min": 0.2`) a přegenerujte tabulky
  (`make tables VEHICLE=…`, `DG_CLAMP_MIN` v `guidance.h` srovnat).
* Linearita: změřte aspoň `k` při pulzu uprostřed. Odchylka do ±10 % nevadí (plánovač
  přeplánuje každých 100 ms a tah měří); víc → upravte mechaniku, firmware mapuje lineárně.

### 9.5 Detekce tahu `g.onset_bump`

Navedení pozná zážeh podle skoku osového zrychlení o `g.onset_bump` (4 m/s²) dva vzorky po
sobě. Ze záznamu akcelerometru při statickém zážehu (TriSense na stojanu, nebo spočítat
`F/m` z tenzometru): `g.onset_bump` ≈ 1/3 zrychlení 30 ms po začátku tahu, ale aspoň 3×
šum akcelerometru ve volném pádu (~0,5 m/s²). Výchozí 4 m/s² pro ~100 N / 2,85 kg.

### 9.6 Detekce startu `mst.launch_g`

Polovina průměrného zrychlení hlavního motoru v prvních 0,2 s (datasheet motoru / tah ÷
hmotnost), nejméně 2 g. Breakwire je primární, tohle je záloha.

---

## 10. Po letu

### 10.1 Stažení letového záznamu

Master zapisuje na flash 200× za s za letu a 10× za s na zemi (96 B/záznam). Nejjednodušší
je záložka diagnostiky **Letový záznam** (*Najít lety* → *Stáhnout vybrané lety* — souhrn
kalibračních čísel se ukáže nad grafy, *Exportovat CSV*). Z příkazové řádky (diagnostiku
zavřete, raketa v IDLE nebo RECOVERY):

```bash
python tools/daidalos_log.py --port COM5 --out let1.csv --raw let1_raw.txt
```

Program vypíše souhrn kalibračních čísel na konci každého letu. Po stažení a zálohování
můžete záznam smazat (parametry zůstanou):

```bash
python tools/daidalos_log.py --port COM5 --erase
```

Konzolové příkazy, které program používá: `LOG INFO`, `LOG FLIGHTS`, `LOG DUMP [od [počet]]`, `LOG STOP`,
`LOG ERASE`.

### 10.2 Cd → `g.cd_body`

Sloupec `drag` = měřený / modelový odpor na konci volného pádu (už s aerobrzdou, jak ji
raketa opravdu měla). Je to poměr pro celý pád, tedy pro `cd_body + fin_dcd_brake`:

```
g.cd_body_nový = (g.cd_body + g.fin_dcd_brake) × drag − g.fin_dcd_brake
```

Pak přepočítat pásmo (kap. 8.8). Hodnota `drag` mimo 0,7–1,4 znamená, že něco v modelu
neplatí (hmotnost? brzda neotevřená? Cd odhadnuté z modelu zádí napřed?) — zjistěte proč,
než to „opravíte“ číslem.

### 10.3 Účinnost křidélek → `fin.gain0`

Sloupec `fin_gain` = odhad skutečné síly křidélek / model na konci letu. Pro další let
`fin.gain0 = fin_gain` (adaptace pak začne blíž pravdě). Hodnota < 0,6 nebo > 1,6: zkontrolujte
`srv.fin_us_per_deg` a vůle v pákách.

### 10.4 Tah → `g.thrust_scale`

Sloupec `tscale` během hoření (`nav_phase` 4). Medián × `g.thrust_scale` = nový odhad. Jeden
let je jeden motor — stejně jako u statických zážehů průměrujte přes víc letů.

### 10.5 Časování

Ze sloupce `events` (bity sekvenceru) a `t_event`: aerokryt má být 0,6 s před zážehem, nohy
1,5–2,2 s před dosednutím (`USER_MANUAL.md` kap. 5.6), lidar ≤ 0,3 s po nohách. Pokud se
nohy otevírají pozdě pro vaše nohy, prodlužte `seq.legs_lead_s`.

---

## 11. Pořadí a záznamový list

**Pořadí** (každý krok staví na předchozím):

1. Elektrika (kap. 7) — bez správného napětí a kontinuity nemá smysl pokračovat.
2. Akcelerometr a magnetometr (3.2, 3.3) — deska po desce, příklady knihovny.
3. Serva: střed → strmost → **znaménka** (6.2–6.8).
4. Vážení, těžiště, setrvačnost, geometrie (8.1–8.5).
5. Statický zážeh: tahová křivka, zpoždění roznětu, klapka, detekce tahu (kap. 9).
6. LandSim: pásmo, odpor křidélek, případně zisky (8.7, 8.8, 12) → odeslat → uložit do flash.
7. Lidar (kap. 4) a UWB anténa (5.3).
8. Na místě startu: kotvy UWB (5.1), deklinace, hmotnost znovu, *Kalibrace gyra*, *Pad
   reference*.
9. Po letu: záznam → Cd, účinnost křidélek, tah (kap. 10) → přepočítat pásmo.

**Záznamový list** — vytiskněte a vyplňte:

| hodnota | změřeno | jednotka | datum | kdo | poznámka |
|---|---|---|---|---|---|
| `ACC_OFFSET` / `ACC_SCALE` | | g / – | | | deska č. |
| `MAG_HARD_IRON` | | µT | | | v sestavě s baterií |
| `MAG_DECLINATION_DEG` | | ° | | | místo |
| `pos.gps_latency_ms` | | ms | | | |
| `pos.lidar_offset_m` | | m | | | |
| `pos.lidar_tilt_x_deg` / `_y_deg` | | ° | | | metoda A/B |
| `pos.lidar_delay_ms` | | ms | | | snímků při 240 fps |
| kotvy 1–4 (x, y, z) | | m | | | diagonály: |
| `srv.tvc_x_trim_us` / `_y_trim_us` | | µs | | | |
| `srv.tvc_us_per_deg` | | µs/° | | | θ trysky při ±100 µs |
| `srv.tvc_x_sign` / `_y_sign` | | ±1 | | | **ověřeno 2 lidmi** |
| `srv.clamp_closed_us` / `_open_us` | | µs | | | k při closed: |
| `srv.fin1..4_trim_us` | | µs | | | |
| `srv.fin_us_per_deg` | | µs/° | | | |
| `srv.fin1..4_sign` | | ±1 | | | **ověřeno 2 lidmi** |
| brzda 4 × 15° | | ° | | | rozptyl: |
| `srv.cover_lock_us` / `_unlock_us` | | µs | | | |
| `VBAT_DIVIDER` | | – | | | |
| `ACS712_ZERO_V` / `_V_PER_A` | | V / V/A | | | |
| `CONT_THRESHOLD` (A_ok / A_open) | | ADC | | | |
| `g.mass_gross_kg` | | kg | | | **den startu** |
| `g.mass_prop_kg` | | kg | | | |
| těžiště od špičky | | m | | | |
| `fin_arm_m` | | m | | | |
| `inertia.transverse` / `.roll` | | kg·m² | | | T, L, d: |
| `g.cd_body` | | – | | | zdroj: |
| `g.fin_dcd_zero` / `_brake` | | – | | | LandSim commit: |
| impulz motoru / `g.thrust_scale` | | N·s / – | | | motor č. |
| zpoždění roznětu (max) | | s | | | typ roznětu |
| `g.onset_bump` | | m/s² | | | |
| `mst.launch_g` | | g | | | |

---

## 12. Když se raketa změní

| co se změnilo | co udělat |
|---|---|
| hmotnost o pár % | `g.mass_gross_kg` → přepočítat pásmo (8.8) → odeslat → uložit |
| hmotnost o > 10 %, jiné rozložení hmoty | navíc setrvačnost (8.4), **přefitovat zisky**: v záložce Zážeh zaškrtnout *Přefitovat zisky* (minuty až desítky minut), odeslat i zisky |
| jiný Cd (nový nos, jiný povrch) | `g.cd_body` (8.6) → pásmo; po letu z měření (10.2) |
| jiná křidélka (tvar, velikost, rameno) | 8.3, 8.5 → LandSim (ΔCd přepočítá sám) → **přegenerovat tabulky** (`make tables VEHICLE=…`, geometrie křidélek je v `guidance_tables.h`) → znovu 6.6, 6.7 |
| jiný motor | 9.1–9.5 → `make tables VEHICLE=…` → nahrát Navigation MCU |
| jiné serva / páky | 6.2–6.7 včetně **znamének** |
| přemontovaná elektronika | 3.3 (magnetometr), 3.6 (orientace), 6.4 a 6.7 (znaménka) |
| nové místo startu | 3.4 (deklinace), 5.1 (kotvy), 4.4 (dosah lidaru na tamním povrchu) |

Po každé změně: `cd test && make check` (pokud se měnil kód nebo tabulky), v diagnostice
„Parametry: synchronní“, export profilu parametrů, zápis do záznamového listu.
