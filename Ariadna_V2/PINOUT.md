# Daidalos / Ariadna V2 — Hardware & Pinout Reference (All 4 PCBs)

Complete, **netlist-verified** hardware reference for all four PCBs in the Daidalos
ecosystem (**fw 3.34**):

1. **Ariadna V2 Flight Computer** (`Netlist_Ariadna_V2_2026-11-19.enet`) — Master + Position + Navigation
2. **UWB Anchor / Tag PCB** (`Netlist_Anchor-PCB_2026-11-19.enet`) — identical PCB for 4× ground anchors & onboard UWB tag
3. **Ground Station PCB** (`Netlist_PCBGroundStation_2026-11-19.enet`) — populated: Pico 2W, LoRa Ra-02, OLED SSD1306, 3 buttons
4. **Rampa PCB1 — Launch Pad Controller** (`RampaNetlist_PCB1_2026-11-19.enet`) — Pico 2, umbilical I2C slave, dual-stage pyros, load cell, LoRa

**This file is the reference; the sketches are the truth.** Every pin, I²C address,
divider ratio, and active polarity below was extracted directly from the EasyEDA Pro
netlists and cross-checked against the firmware.

---

## 0. AI / Quick-Lookup Hardware Map (Token-Optimized)

```yaml
# Exact netlist-verified hardware map for rapid human & AI lookup (fw 3.34)
boards:
  ariadna_master: # Netlist_Ariadna_V2 | Pico 2 W (RP2350) | Master_MCU/Master_MCU.ino
    uart0_pos: {tx: GP0, rx: GP1, baud: 921600, note: "GP0 also wired to NAV.D11"}
    uart1_nav: {tx: GP4, rx: GP5, baud: 921600}
    i2c0_bus:  {sda: GP8, scl: GP9, clk_khz: 400, pullup: "10k to 3V3M",
                devs: {pca9685: 0x40}, note: "PCA9685 servos ONLY — keep umbilical off this bus!"}
    spi0_lora: {miso: GP16, cs: GP17, sck: GP18, mosi: GP19, dio0: GP20, rst: GP21, ic: "Ra-02 SX1278 433MHz"}
    spi1_flash:{miso: GP12, cs: GP13, sck: GP14, mosi: GP15, ic: "W25Q128JV 16MB"}
    io:        {rgb_sk6812: GP2, buzzer: GP3, pca_oe_low: GP6, nav_rst_low: GP7,
                breakwire_pu: GP10, aux_s16: GP22}
    adc:       {i_acs712_20a: "GP26 (10k/10k 2:1, 0A=1.252V)", vbat_2s: "GP27 (20k/10k 3:1)",
                rbf: "GP28 (Q4 IRF9540NS gate network, removed=LOW)"}
    gotchas:   ["GP6 connects to PCA9685 /OE AND via diode D6 to POS.RUN (PRST)",
                "Do NOT plug Rampa pogo into Master I2C1/I2C2 (shared with PCA9685); use Navigation I2CN instead"]

  ariadna_position: # Netlist_Ariadna_V2 | XIAO RP2350 | Position_MCU/Position_MCU.ino
    spi0_imu:  {sck: D8, miso: D9, mosi: D10, cs: D3, ic: "TriSense Pro ONLY (drives SCK/MOSI when CS=H!)"}
    i2c1_sens: {sda: D4, scl: D5, clk_khz: 400,
                devs: {ak09918c: 0x0C, tfluna_lidar: 0x10, uwb_tag_I2CP: 0x2A, bmp580: 0x46}}
    uart_mst:  {tx: D6, rx: D7, baud: 921600}
    pio_gps:   {tx: D11, rx: D12, conn: uGPS1, ic: "LK76 NMEA 115200 5Hz"}
    rgb:       {pin: "XIAO onboard PIN_NEOPIXEL (22)", pwr: "NEOPIXEL_POWER (23)"}

  ariadna_navigation: # Netlist_Ariadna_V2 | XIAO RP2350 | Navigation_MCU/Navigation_MCU.ino
    uart_mst:  {tx: D6, rx: D7, baud: 921600}
    pyro_out:  {ch1_ign1: D3, ch2_ign2: D8, ch3_legs: D9, fet: "AO3400A low-side, HIGH=FIRE"}
    pyro_cont: {ch1: D0, ch2: D1, ch3: D2, divider: "20k/10k 3:1 on PYRO1..3"}
    i2c1_umb:  {sda: D4, scl: D5, clk_khz: 100, conn: I2CN, devs: {rampa_umbilical: 0x2B},
                note: "DEDICATED umbilical bus (isolated from servos!); Wire1.end()+Hi-Z at T-100ms; no PCB pull-ups (uses internal PU, 4.7k on harness recommended >15cm)"}
    rgb:       {pin: "XIAO onboard PIN_NEOPIXEL (22)", pwr: "NEOPIXEL_POWER (23)"}

  uwb_anchor_and_tag: # Netlist_Anchor-PCB | XIAO RP2350 | Uwb_Anchor.ino / Uwb_Tag.ino
    spi0_uwb:  {sck: D8, miso: D9, mosi: D10, cs: D2, int: D0, rst_od: "D1 (OPEN DRAIN! 10k R9 to GND)", ic: "DWM1000"}
    i2c1_tag:  {sda: D4, scl: D5, pullup: "10k R2/R3", conn: "I2CP/I2CP1 uSup", slave_addr: 0x2A,
                frames: "reg 0x00=74B full (0xA7), reg 0x01=34B fast (0xA8, 0.79ms @ 400kHz)"}
    rgb:       {pin: "D3 (external SK6812 RGB1 via 200R R23, powered from 3V3 -> pwr=0xFF)"}
    power:     "1S -> SW1 -> AP7361C-33 (3V3). Tag mode: powered via I2CP cable (no 1S, or cut jumper R)"

  ground_station: # Netlist_PCBGroundStation | Pico 2 W (RP2350) | Ground_Station/Ground_Station.ino
    populated: ["Pico 2W (RP_M1)", "LoRa Ra-02 (LORA1)", "OLED SSD1306 (U3)", "Buttons BL/BC/BR"]
    unpopulated: ["U2 XIAO-ESP32-C6", "GPS1", "SIM800L", "CARD1 microSD", "BUZZER1", "RGB1", "TP4054", "RBF1", "SW1"]
    i2c0_oled: {sda: GP4, scl: GP5, clk_khz: 400, pullup: "10k R28/R29", addr: 0x3C, ic: "SSD1306 HS96L03W2C03"}
    spi1_lora: {miso: GP12, cs: GP11, sck: GP14, mosi: GP15, rst: GP20, dio0: GP21,
                note: "Firmware uses RP2350 SPI1 HW pins (MISO=GP12, CS=GP11); check schematic label quirk on LORA1.13/15"}
    buttons:   {BL_test_on: GP3, BC_test_off: GP6, BR_reset: GP10,
                polarity: "ALL ACTIVE HIGH (switch to 3V3, 10k R31/R32/R33 pull-down to GND)"}

  rampa_pcb1: # RampaNetlist_PCB1 | Pico 2 (RP2350, U7) | Rampa_MCU/Rampa_MCU.ino
    i2c0_umb:  {sda: "GP0 (R21 1k -> J3.4)", scl: "GP1 (R22 100R -> J3.3)", slave_addr: 0x2B,
                note: "Detaches (Wire.end + Hi-Z INPUT) at T-100ms after RAMPA_CMD_COMMIT"}
    i2c1_int:  {sda: GP6, scl: GP7, clk_khz: 400, pullup: "4.7k R18/R19",
                devs: {tca9534: 0x20, ina226_vbat: 0x40, ina226_umb5v: 0x41, ads1115: 0x48}}
    tca9534_0x20: {P0: "BUZZER_EN (OUT H)", P1: "ROCKET_EN 5V buck (OUT H)", P2: "RBF_SENSE key (IN H)",
                   P3: "LORA_RST (OUT L)", P4: "LORA_EN U12 pwr (OUT H; LOW=OFF on umbilical)",
                   P5: "LORA_DIO0 (IN)", P6: "FIRE_LED (OUT H)", P7: "BTN SW1 abort (IN L, 10k PU)"}
    ads1115_0x48: {AIN0: "CONT1 (10k/4.7k)", AIN1: "CONT2", AIN2: "CONT3", AIN3: "5V_SENS/2"}
    pyro_out:  {arm_highside: "GP19 (Q21->Q20 AO4407A)", ch1: GP16, ch2: GP17, ch3: GP18,
                bleed: "VPYRO_KEY -> R26(10k) -> D5 -> VPYRO_ARMED (<0.8mA safe continuity)"}
    spi0_log:  {sck: GP2, mosi: GP3, miso: GP4, flash_cs: "GP5 (W25Q128)", ads1220_cs: GP15, ads1220_drdy: GP21}
    spi1_lora: {sck: GP10, mosi: GP11, miso: GP12, cs: GP13, pwr_gate: "U12 TPS22918 via TCA9534.P4"}
    other:     {servos_J12: [GP8, GP9, GP14, GP22], rgb_sk6812: GP20, pressure_J9: "GP26 (4.7k/10k)",
                umb2_spare_J14: [GP27, GP28], loadcell_J8: "ADS1220 AIN0/1", therm_J10_J11: "ADS1220 AIN2/3"}
```

---

## The Boards

| Board | MCU | Role | Sketch |
|---|---|---|---|
| **Control / Master** | Raspberry Pi Pico 2 W | mission state machine, servos (PCA9685), flash log, bidirectional LoRa | `Master_MCU/Master_MCU.ino` |
| **Position** | Seeed XIAO RP2350 | sensor fusion, navigation KF, UWB multilateration, GPS UTC clock | `Position_MCU/Position_MCU.ino` |
| **UWB Tag** | Seeed XIAO RP2350 (Anchor PCB) | two-way ranging, I²C slave `0x2A` (34 B fast frame) | `Uwb_Tag/Uwb_Tag.ino` |
| **Navigation** | Seeed XIAO RP2350 | guidance, ignition timing, TVC, fins, pyros, **Rampa umbilical I²C master (`I2CN`)** | `Navigation_MCU/Navigation_MCU.ino` |
| **Ground Station** | Raspberry Pi Pico 2 W | bidirectional LoRa (10 s + T-1 s TDMA), OLED, USB to visualizer | `Ground_Station/Ground_Station.ino` |
| **UWB Anchor ×4** | Seeed XIAO RP2350 (Anchor PCB) | fixed ground ranging beacons | `Uwb_Anchor/Uwb_Anchor.ino` |
| **Rampa PCB1** | Raspberry Pi Pico 2 | launch pad controller, umbilical I²C slave `0x2B`, dual-stage igniters | `Rampa_MCU/Rampa_MCU.ino` |

---

## 1. Control / Master MCU — Raspberry Pi Pico 2 W (`Netlist_Ariadna_V2`)

Pin numbers are raw `GPnn` GPIO numbers (`Master_MCU/Master_MCU.ino`).

| Pin | Function | Peripheral / Net | Constant |
|---|---|---|---|
| GP0 | UART0 TX | Position MCU RX (`D7`) + Navigation `D11`, 921600 baud | `PIN_POS_TX` |
| GP1 | UART0 RX | Position MCU TX (`D6`), 921600 baud | `PIN_POS_RX` |
| GP2 | PIO / PWM | SK6812 RGB annunciator (`RGB1` via `R23` 200 Ω) | `ARIADNA_RGB_PIN` |
| GP3 | PWM | Buzzer (`BUZZER1` via `R24` 200 Ω) | `PIN_BUZZER` |
| GP4 | UART1 TX | Navigation MCU RX (`D7`), 921600 baud | `PIN_NAV_TX` |
| GP5 | UART1 RX | Navigation MCU TX (`D6`), 921600 baud | `PIN_NAV_RX` |
| GP6 | digital out | PCA9685 `/OE` (`R25` 10 kΩ PU; LOW = outputs on) + diode `D6` → `PRST` | `PIN_PCA_OE` |
| GP7 | digital out | RESET to Navigation MCU via diode `D7` → `NRST` | `PIN_RST_NAV` |
| GP8 | I2C0 SDA | **PCA9685 (`0x40`) ONLY** (`R21` 10 kΩ; do NOT plug umbilical into `I2C1`/`I2C2`) | `PIN_I2C_SDA` |
| GP9 | I2C0 SCL | same bus, `R22` 10 kΩ | `PIN_I2C_SCL` |
| GP10 | digital in, PU | Breakwire (`TRH1`/`TRH2`, `R26` 10 kΩ) — launch detect | `PIN_BREAKWIRE` |
| GP11 | — | free | — |
| GP12 | SPI1 MISO | W25Q128 flash (`U32.2` DO) | `PIN_FLASH_MISO` |
| GP13 | SPI1 CS | W25Q128 flash (`U32.1` /CS) | `PIN_FLASH_CS` |
| GP14 | SPI1 SCK | W25Q128 flash (`U32.6` CLK) | `PIN_FLASH_SCK` |
| GP15 | SPI1 MOSI | W25Q128 flash (`U32.5` DI) | `PIN_FLASH_MOSI` |
| GP16 | SPI0 MISO | LoRa Ra-02 (`LORA1.13`) | `PIN_LORA_MISO` |
| GP17 | SPI0 CS | LoRa Ra-02 (`LORA1.15` NSS) | `PIN_LORA_CS` |
| GP18 | SPI0 SCK | LoRa Ra-02 (`LORA1.12`) | `PIN_LORA_SCK` |
| GP19 | SPI0 MOSI | LoRa Ra-02 (`LORA1.14`) | `PIN_LORA_MOSI` |
| GP20 | digital in | LoRa Ra-02 `DIO0` (`LORA1.5`) | `PIN_LORA_DIO0` |
| GP21 | digital out | LoRa Ra-02 `RST` (`LORA1.4`) | `PIN_LORA_RST` |
| GP22 | digital out | `S16` (`SR1_2S2.12`) — spare output | `PIN_AUX_S16` |
| GP26 / ADC0 | analog in | ACS712-20A current (`AM.7` via `R4`/`R3` 10k/10k 2:1 divider, 0 A = 1.252 V) | `PIN_I_SENSE` |
| GP27 / ADC1 | analog in | 2S battery voltage (`R2`/`R1` 20k/10k 3:1 divider) | `PIN_V_BAT` |
| GP28 / ADC2 | digital/analog in | RBF pin (`Q4` IRF9540NS gate + `R5`/`R6`/`R7` 10k network; removed = LOW) | `PIN_RBF` |
| USB | USB CDC | Diagnostic console (`Diagnostics/daidalos_diag.py`) | `Serial` |

> **Bench example, not flight:** `Examples/Static_Fire_Link` puts the Static Fire Stand on this same bus
> as an I²C slave at `0x2C` (stand GP4/GP5 + GND to `I2C1`/`I2C2`; never the 3V3 pins). The flight
> firmware does not use it, and the Rampa umbilical stays on the Navigation `I2CN` as above.

### PCA9685 Servo Channels (`U33`, addr `0x40`, 339 Hz digital servos only)

Default channel map (editable via `srv.ch_*` parameters in flash):
- `0–4`, `13–15`: unused in flight (TEST mode only)
- `5`: TVC axis 1 (`CH_TVC_X`), `6`: TVC axis 2 (`CH_TVC_Y`)
- `7`: thrust clamp (`CH_CLAMP`), `8`: aerocover latch (`CH_COVER`)
- `9–12`: fins 1–4 (`CH_FIN_0..3`)
- Power rail jumpers on PCB: `S1-4` rail (`SP`) selectable via `J2S` (2S) or `J5V` (+5V LM1085); `S5-8`, `S9-13`, `SR1_2S2` powered from `2S`.

---

## 2. Position MCU — Seeed XIAO RP2350 (`Netlist_Ariadna_V2`)

| Pin | Function | Peripheral / Net | Constant |
|---|---|---|---|
| D0–D2 | — | free (legacy onboard DWM1000 footprint unpopulated) | — |
| D3 | digital out | Voltino TriSense Pro CS (`U28.7`) | `PIN_TRISENSE_CS` |
| D4 | I2C1 SDA (400 kHz) | `PSDA`: BMP580 (`0x46`), AK09918C (`0x0C`), TF-Luna (`0x10`, `Lidar1.2`), UWB Tag (`0x2A`, `I2CP.3`) | `PIN_I2C_SDA` |
| D5 | I2C1 SCL (400 kHz) | `PSCL`: same bus (`Lidar1.3`, `I2CP.4`) | `PIN_I2C_SCL` |
| D6 | UART0 TX | Master MCU RX (`GP1`), 921600 baud | `PIN_MASTER_TX` |
| D7 | UART0 RX | Master MCU TX (`GP0`), 921600 baud | `PIN_MASTER_RX` |
| D8–D10 | SPI0 SCK/MISO/MOSI | **TriSense ONLY** (SN74LVC2T45 drives SCK/MOSI whenever CS=HIGH!) | `PIN_SPI_*` |
| D11 | PIO UART TX | LK76 GPS RX (`uGPS1.3`), 115200 baud 5 Hz | `PIN_GPS_TX` |
| D12 | PIO UART RX | LK76 GPS TX (`uGPS1.4`) — also supplies UTC clock to Master (`ARIADNA_PKT_GPS_TIME`) | `PIN_GPS_RX` |
| onboard | PIO | XIAO WS2812 (`PIN_NEOPIXEL = 22`, power `NEOPIXEL_POWER = 23`) | `ARIADNA_RGB_PIN` |

---

## 3. Navigation (Guidance) MCU — Seeed XIAO RP2350 (`Netlist_Ariadna_V2`)

| Pin | Function | Peripheral / Net | Constant |
|---|---|---|---|
| D0–D2 | analog in | Pyro continuity ch 1–3 (`PYRO1..3` via 20k/10k 3:1 dividers `R13/14`, `R16/17`, `R19/20`) | `PIN_CONT_1..3` |
| D3 | digital out | **Pyro 1 — landing motor igniter** (`Q1` AO3400A, `R12` 10k PD) | `PIN_PYRO_1` |
| D4 | **I2C1 SDA (100 kHz)** | **`I2CN` uSup (`NSDA`) — DEDICATED Rampa umbilical (`0x2B`), detached Hi-Z at T-100 ms** | `PIN_UMB_SDA` |
| D5 | **I2C1 SCL (100 kHz)** | **`I2CN` uSup (`NSCL`) — DEDICATED Rampa umbilical (`0x2B`), detached Hi-Z at T-100 ms** | `PIN_UMB_SCL` |
| D6–D7 | UART0 TX/RX | Master MCU (`GP5`/`GP4`), 921600 baud (also bridges `RAMPA_CMD`/`RAMPA_STATUS`) | `PIN_MASTER_TX/RX` |
| D8 | digital out | **Pyro 2 — backup igniter** (`Q2` AO3400A, `R15` 10k PD) | `PIN_PYRO_2` |
| D9 | digital out | **Pyro 3 — landing-leg release** (`Q3` AO3400A, `R18` 10k PD) | `PIN_PYRO_3` |
| onboard | PIO | XIAO WS2812 (`PIN_NEOPIXEL = 22`, power `NEOPIXEL_POWER = 23`) | `ARIADNA_RGB_PIN` |

---

## 4. Ground Station — Raspberry Pi Pico 2 W (`Netlist_PCBGroundStation`)

**Populated components only:** `RP_M1` (Pico 2 W), `LORA1` (Ra-02 SX1278), `U3` (SSD1306 OLED), and 3 buttons (`BL`, `BC`, `BR`).
*(Unpopulated footprints on PCB: `U2` XIAO-ESP32-C6, `GPS1`, `SIM1` SIM800L, `CARD1` microSD, `BUZZER1`, `RGB1`, `TP1` TP4054, `RBF1`, `SW1`.)*

| Pin | Function | Peripheral / Net | Constant |
|---|---|---|---|
| GP3 | digital in, **active HIGH** | `BL` button (`BL.5=3V3`, `BL.6=GPIO3`, `R31` 10 kΩ pull-down to GND) — TEST ON | `TEST_ON_PIN` |
| GP4 | I2C0 SDA (400 kHz) | SSD1306 OLED `U3.4` (`0x3C`) + `I2C1`/`I2C2` uSup (`R29` 10 kΩ PU) | `I2C_SDA` |
| GP5 | I2C0 SCL (400 kHz) | SSD1306 OLED `U3.3` + `I2C1`/`I2C2` uSup (`R28` 10 kΩ PU) | `I2C_SCL` |
| GP6 | digital in, **active HIGH** | `BC` button (`BC.5=3V3`, `BC.6=GPIO6`, `R32` 10 kΩ pull-down to GND) — TEST OFF | `TEST_OFF_PIN` |
| GP10 | digital in, **active HIGH** | `BR` button (`BR.5=3V3`, `BR.6=GPIO10`, `R33` 10 kΩ pull-down to GND) — RESET | `RESET_BTN_PIN` |
| GP11 | SPI1 CS | LoRa Ra-02 `NSS` *(see note below)* | `LORA_CS` |
| GP12 | SPI1 MISO | LoRa Ra-02 `MISO` *(RP2350 HW SPI1_RX; see note below)* | `LORA_MISO` |
| GP14 | SPI1 SCK | LoRa Ra-02 `SCK` (`LORA1.12`) | `LORA_SCK` |
| GP15 | SPI1 MOSI | LoRa Ra-02 `MOSI` (`LORA1.14`) | `LORA_MOSI` |
| GP20 | digital out | LoRa Ra-02 `RST` (`LORA1.4`) | `LORA_RESET` |
| GP21 | digital in | LoRa Ra-02 `DIO0` (`LORA1.5`) | `LORA_DIO0` |
| USB | USB CDC (115200) | Framed telemetry + sync (`0x40`/`0x41`) & text uplink commands (`UTC:`, `CMD:*`) | `Serial` |

> **Netlist note (`LORA1`):** In `Netlist_PCBGroundStation_2026-11-19.enet`, `LORA1.13` (`MISO2`) connects to `RP_M1.17` (GP13) and `LORA1.15` (`GPIO13`) is unrouted to `RP_M1`, whereas RP2350 hardware `SPI1_RX` lives on **GP12**. Firmware keeps `LORA_MISO = 12, LORA_CS = 11` matching the working soldered board. OLED power polarity jumpers `J1`/`J2` select pins 1–2 of `U3` (`HS96L03W2C03`).

---

## 5. UWB Anchor & UWB Tag PCB — Seeed XIAO RP2350 (`Netlist_Anchor-PCB`)

Identical hardware for all 4 ground anchors (`Uwb_Anchor/Uwb_Anchor.ino`, `ANCHOR_ID = 1..4`) and the vehicle's UWB tag (`Uwb_Tag/Uwb_Tag.ino`).

| Pin | Function | Peripheral / Net | Constant |
|---|---|---|---|
| D0 | digital in | DWM1000 `IRQ` (`UWB.22`, polled) | `PIN_UWB_INT` |
| D1 | **open-drain out** | DWM1000 `RSTn` (`UWB.3`, `R9` 10 kΩ to GND) — **never drive HIGH** | `PIN_UWB_RST` |
| D2 | digital out | DWM1000 `CS` (`UWB.17`) | `PIN_UWB_CS` |
| D3 | PIO / NeoPixel | **External SK6812 RGB (`RGB1` via `R23` 200 Ω, powered from `3V3`)** | `ARIADNA_RGB_PIN` (`D3`, `pwr=0xFF`) |
| D4 | I2C1 SDA (slave `0x2A`) | `PSDA` (`I2CP.3`, `I2CP1.3`, `R3` 10 kΩ PU) — Tag mode only | `PIN_I2C_SDA` |
| D5 | I2C1 SCL | `PSCL` (`I2CP.4`, `I2CP1.4`, `R2` 10 kΩ PU) — Tag mode only | `PIN_I2C_SCL` |
| D8–D10 | SPI0 SCK/MISO/MOSI | DWM1000 (`UWB.20/19/18`) | `PIN_SPI_*` |
| D11–D12 | spare | `uGPS1.3/4` | — |

- **Power:** 1S LiPo (`1S+1`) → `SW1` → `U34` (`AP7361C-33`) → jumper `R` → `3V3` → jumper `X` → XIAO `3V3`. In **Tag mode**, board is powered through the `I2CP` uSup cable (do NOT fit a 1S battery unless jumper `R` is cut; unplug `I2CP` before USB flashing).
- **Tag I²C protocol (`common/ariadna_uwb_link.h`):** boots in 74 B full frame (`0xA7`, reg `0x00`); Position MCU writes `0x01` once to switch to the **34 B fast frame** (`0xA8`, `0.79 ms` @ 400 kHz, 1 cm range, 1 dBm `rxp_dbm`, 2 ms age steps).
- **Anchor EUIs (`startAsAnchor(..., false)` mandatory!):** `01:00..04:00:5B:D5:A9:9A:E2:9C` (short addr `0x0001..0x0004`); Tag: `AA:00:5B:D5:A9:9A:E2:9C` (`0x00AA`).

---

## 6. Rampa PCB1 — Launch Pad Controller (`RampaNetlist_PCB1`)

**Target:** Raspberry Pi Pico 2 (`U7`, RP2350), sketch `Ariadna_V2/Rampa_MCU/Rampa_MCU.ino`.

### 6.1 Pico 2 (`U7`) GPIO Map

| Pin | Function | Peripheral / Net | Constant |
|---|---|---|---|
| GP0 | **I2C0 SDA (Slave `0x2B`)** | `UMB_TX` → `R21` (1 kΩ) + `D4B` ESD → **`J3.4`** (Rocket umbilical SDA) | `PIN_UMB_SDA` |
| GP1 | **I2C0 SCL (Slave `0x2B`)** | `UMB_RX` → `R22` (100 Ω) + `R23` (100 kΩ PU) + `D4A` ESD → **`J3.3`** (Rocket umbilical SCL) | `PIN_UMB_SCL` |
| GP2–GP4 | SPI0 SCK/MOSI/MISO | Shared by `U9` W25Q128 flash & `U14` ADS1220 24-bit ADC | `RAMPA_SPI0_*` |
| GP5 | SPI0 CS | `U9` W25Q128JV flash (`FLASH_CS`, `R17` 10 kΩ PU) | `PIN_FLASH_CS` |
| GP6 | **I2C1 SDA (Master 400 kHz)** | Internal bus (`R18` 4.7 kΩ PU): `U10` TCA9534, `U1`/`U8` INA226, `U11` ADS1115 | `PIN_INT_SDA` |
| GP7 | **I2C1 SCL (Master 400 kHz)** | Internal bus (`R19` 4.7 kΩ PU) | `PIN_INT_SCL` |
| GP8, GP9, GP14, GP22 | PWM out | 4× servos (`J12` pins 1, 4, 7, 10 via 100 Ω; powered from `VBAT`) | `PIN_SERVO1..4` |
| GP10–GP13 | SPI1 SCK/MOSI/MISO/CS | `M1` Ra-02 LoRa (via 1 kΩ series resistors; **powered OFF via `U12` while on umbilical**) | `PIN_LORA_*` |
| GP15 | SPI0 CS | `U14` ADS1220 24-bit ADC (`R59` 10 kΩ PU) — load cell `J8` + thermistors `J10`/`J11` | `PIN_ADS1220_CS` |
| GP16 | digital out | **Pyro 1 fire** (`Q22` AO3400A low-side → `J5.2`, `R31` 100 Ω) | `PIN_PYRO1_FIRE` |
| GP17 | digital out | **Pyro 2 fire** (`Q23` AO3400A low-side → `J6.2`, `R36` 100 Ω) | `PIN_PYRO2_FIRE` |
| GP18 | digital out | **Pyro 3 fire** (`Q24` AO3400A low-side → `J7.2`, `R41` 100 Ω) | `PIN_PYRO3_FIRE` |
| GP19 | digital out | **Pyro Master ARM** (`Q21` → `Q20` AO4407A high-side P-FET → `VPYRO_ARMED` on `J5..J7.1`) | `PIN_PYRO_ARM` |
| GP20 | PIO / NeoPixel | SK6812 RGB LED (`LED1` via `R56` 100 Ω) | `ARIADNA_RGB_PIN` |
| GP21 | digital in | `U14` ADS1220 `DRDY` | `PIN_ADS1220_DRDY` |
| GP26 / ADC0 | analog in | Pressure transducer (`J9.2` via `R46`/`R47` 4.7k/10k + Schottky clamp `D6A/B`) | `PIN_PRESSURE_ADC` |
| GP27–GP28 | spare | `J14` 2nd umbilical port (`UMB_TX2`/`UMB_RX2`, ESD-protected; non-matching HW I²C pair) | — |

### 6.2 Rampa Internal I²C1 Bus (`Wire1`, GP6/GP7, 400 kHz)

| Address | IC | Purpose & Channel Assignment |
|---|---|---|
| `0x20` | `U10` TCA9534PWR | `P0`: `BUZZER_EN` (OUT H), `P1`: `ROCKET_EN` 5V buck `U2` (OUT H), `P2`: `RBF_SENSE` key `J4` (IN H), `P3`: `LORA_RST` (OUT L), `P4`: `LORA_EN` `U12` LoRa power switch (OUT H; **kept LOW when umbilical active**), `P5`: `LORA_DIO0` (IN), `P6`: `FIRE_LED` (OUT H), `P7`: `BTN` `SW1` local abort (IN **active LOW**, 10 kΩ PU) |
| `0x40` | `U1` INA226 | Main battery `VBAT` monitor (`R4` 5 mΩ shunt, `1.25 mV/LSB`) |
| `0x41` | `U8` INA226 | Umbilical `UMB_5V` rail monitor (`R20` 5 mΩ shunt, `EN_5VR` alert) |
| `0x48` | `U11` ADS1115 | `AIN0..2`: `CONT1..3` (`10 kΩ / 4.7 kΩ` divider; ~1.4 V with igniter, ~0 V open via `R26` 10 kΩ + `D5` < 0.8 mA bleed), `AIN3`: `5V_SENS / 2` |

### 6.3 Rampa Safety Interlocks & Power

- Reverse-polarity / over-voltage protection (`Q1`/`Q2` AO4407A + `U13` TLV6700 + `F1` 10 A fuse + `D1` SMAJ10A).
- Physical key switch on **`J4`** (in series with `F2` 5 A fuse between `VBAT` and `VPYRO_KEY`). Sensed via `R24/R25` (100k/100k) on `U10.P2` (`RBF_SENSE`).
- Firing requires **all three** simultaneously: physical key `J4` closed + `PIN_PYRO_ARM` (GP19) HIGH + `PIN_PYROx_FIRE` (GP16–18) HIGH.

---

## 7. Inter-Board Wiring & Magnetic Pogo Umbilical

### Internal Ariadna V2 Links (921600 baud, 8N1)

```text
   Position MCU (XIAO)          Master MCU (Pico 2 W)          Navigation MCU (XIAO)
        D6  TX  ────────────────►  GP1  RX
        D7  RX  ◄────────────────  GP0  TX  ──────────────────►  D11 (shared TX)
                                   GP4  TX  ──────────────────►  D7  RX
                                   GP5  RX  ◄──────────────────  D6  TX
                                   GP7  (D7 reset) ───────────►  RUN (NRST)
```

### Rocket ↔ Rampa Umbilical (Magnetic Pogo, 3 wires ONLY)

Connect the magnetic pogo harness to **Navigation MCU `I2CN` uSup** (NOT Master `I2C1`/`I2C2`!) so the umbilical is **100% electrically isolated from the PCA9685 servo bus**.

| Ariadna V2 Navigation (`I2CN` uSup) | Magnetic Pogo | Rampa PCB1 (`J3`) | Note |
|---|---|---|---|
| Pin 1 (`GND`) | Pin 1 (`GND`, longest / first-mate-last-break) | Pin 1 (`GND`) | Common ground |
| Pin 2 (`3V3`) | **NOT CONNECTED** | Pin 2 (`UMB_5V`) | Never short Ariadna `3V3` to Rampa `5V`! |
| Pin 3 (`NSDA` / `D4` = GP6) | Pin 2 (`SDA`) | Pin 4 (`J3.4` → `R21` 1k → GP0) | Dedicated `Wire1 SDA` (100 kHz) |
| Pin 4 (`NSCL` / `D5` = GP7) | Pin 3 (`SCL`) | Pin 3 (`J3.3` → `R22` 100R → GP1) | Dedicated `Wire1 SCL` (100 kHz) |

> **Pull-up note:** `NSDA`/`NSCL` have no external pull-up resistors on Ariadna V2. Firmware enables RP2350 internal pull-ups (~50 kΩ) and runs `Wire1` at **100 kHz** (works on short pogo cables). For umbilical cables longer than ~15 cm, add two **4.7 kΩ pull-ups to `3V3`** (`I2CN` pin 2) on the rocket-side pogo adapter.

---

## 8. Bidirectional LoRa TDMA & Launch Sequence (fw 3.34)

Defined in [`common/ariadna_link.h`](common/ariadna_link.h) and [`common/ariadna_rampa_link.h`](common/ariadna_rampa_link.h):

1. **≤ 5 ms Time Synchronization (Rocket ↔ Ground Station ↔ Rampa):**
   - Position MCU parses NMEA UTC time (`$GNRMC`/`$GNGGA`) and sends `ARIADNA_PKT_GPS_TIME` (1 Hz) to Master MCU; PC sends `UTC:<ms>` every 5 s to Ground Station.
   - Master MCU broadcasts a 28-byte `ariadna_lora_sync_t` beacon (1 Hz on ground, 2 Hz during countdown, plus immediate ACK after every RX window) carrying the exact `next_rx_in_ms` from TX start, `t_launch_rem_ms`, and Rampa status/faults.
   - Ground Station anchors its RX-window clock to `gs_rx_ms + next_rx_in_ms - sync_airtime_ms` (**±2 ms lock**, working even indoors without GPS!), with PC UTC 10 s boundary fallback if downlink fades.
   - Master MCU bridges 20 Hz Rampa polls over 921600-baud UART1 to Navigation MCU (`I2CN` / `Wire1` @ 100 kHz), locking Rampa's firing clock to **< 1 ms**.
2. **Periodic 10 s RX Windows + Mandatory T-1 s Final Abort Window:**
   - Every **10 s** (on UTC 10 s boundaries when GPS UTC is valid), Master MCU pauses downlink TX (70 ms guard band) and switches SX1278 to RX for **300 ms**.
   - Ground Station transmits 20-byte `ariadna_lora_uplink_t` (29 ms airtime) at `+55 ms` into the window (and a redundant 2nd copy at `+165 ms` for `ABORT` / `COUNTDOWN`).
   - Supported uplink commands: `PING`, `LAUNCH_SOON` (PREP), `COUNTDOWN <sec>`, `ABORT`, `REBOOT` (**strictly blocked in flight!**), `PADREF`, `DISARM` — all protected by CRC-16 + per-command safety tokens.
   - During an active countdown, Rocket **always** opens a mandatory 300 ms **final abort window centered at T-1.000 s** (`T-1150 ms .. T-850 ms`), regardless of the 10 s grid.
3. **Safe Dual-End Hot-Disconnect before Liftoff (`T - 100 ms`):**
   - At `T - 100 ms`, Master sends `RAMPA_CMD_COMMIT` via Navigation MCU (`I2CN`) and receives `RAMPA_STATE_COMMITTED` (~4 ms round-trip).
   - **Both sides immediately detach hardware I²C:** Rampa calls `Wire.end()` + floats `GP0`/`GP1` (`INPUT` Hi-Z), and Navigation MCU calls `Wire1.end()` + floats `D4`/`D5` (`INPUT` Hi-Z). Master's `I2C0` (`PCA9685` servos) is completely isolated and never touched.
   - At `T - 0`, Rampa re-verifies key + abort button + VBAT and fires the igniter; magnetic pogo detaches cleanly without any possibility of disturbing servos or sensors.

---

## 9. Annunciator Quick Reference

| Board | LED / Buzzer | Pin | Meaning |
|---|---|---|---|
| **Master** | SK6812 + buzzer | GP2 / GP3 | Mission state & startup verdict |
| **Position** | XIAO WS2812 | GP22 (pwr GP23) | Navigation filter & UWB lock (wait for solid green) |
| **Navigation** | XIAO WS2812 | GP22 (pwr GP23) | Pyro readiness & guidance phase |
| **Anchor / Tag** | External SK6812 (`RGB1`) | **D3** (`pwr=0xFF`) | DWM1000 ranging & I²C health |
| **Rampa PCB1** | SK6812 + `FIRE_LED` + buzzer | GP20 / TCA9534 `P6`,`P0` | Blue=idle, Green=ready, Amber fast=countdown, Red=commit/fire, Magenta=abort |
