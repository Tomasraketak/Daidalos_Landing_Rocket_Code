# Daidalos / Ariadna V2 — complete avionics pinout

Every electrical connection in the vehicle, the ground station, and the UWB
anchors, in one place.

**This file is the reference. The sketches are the truth.** Each table names the
source file and the constant that actually drives the pin, so a disagreement can
be resolved in one grep instead of an argument. If you change a pin, change it in
the sketch *and* here — a pinout document that has quietly drifted from the
firmware is worse than none at all, because people stop checking the firmware.

Derived from the pinout spreadsheet, sheet **"Přiřazení pinů 2"**.

---

## The boards

| board | MCU | role | sketch |
|---|---|---|---|
| **Control / Master** | Raspberry Pi Pico 2 W | mission state machine, all servos, flash log, LoRa, arming | `Master_MCU/master_mcu.ino` |
| **Position** | Seeed XIAO RP2350 | sensor fusion, the navigation solution, UWB tag | `Position_MCU/position_mcu.ino` |
| **Guidance / Navigation** | Seeed XIAO RP2350 | guidance, ignition timing, TVC, pyros | `Navigation_MCU/navigation_mcu.ino` |
| **Ground station** | Raspberry Pi Pico 2 | LoRa receive, OLED, USB uplink to the laptop | `Ground_Station/ground_station.ino` |
| **UWB anchor ×4** | Seeed XIAO RP2350 | fixed ground ranging beacon | `Uwb_Anchor/uwb_anchor.ino` |

---

## 1. Control MCU — Raspberry Pi Pico 2 W

Pin numbers are raw `GPnn` GPIO numbers, which is what the arduino-pico core uses
for this board.

| pin | function | peripheral | constant |
|---|---|---|---|
| GP0 | UART0 TX | Position MCU link, 921600 baud | `PIN_POS_TX` |
| GP1 | UART0 RX | Position MCU link | `PIN_POS_RX` |
| GP2 | PIO / PWM | SK6812 RGB annunciator | `PIN_RGB`, `ARIADNA_RGB_PIN` |
| GP3 | PWM | buzzer | `PIN_BUZZER` |
| GP4 | UART1 TX | Navigation MCU link, 921600 baud | `PIN_NAV_TX` |
| GP5 | UART1 RX | Navigation MCU link | `PIN_NAV_RX` |
| GP6 | digital out | RESET line to the Position MCU | `PIN_RST_POS` |
| GP7 | digital out | RESET line to the Navigation MCU | `PIN_RST_NAV` |
| GP8 | I2C0 SDA | PCA9685 servo driver | `PIN_I2C_SDA` |
| GP9 | I2C0 SCL | PCA9685 servo driver | `PIN_I2C_SCL` |
| GP10 | digital in, pull-up | breakwire — launch detect | `PIN_BREAKWIRE` |
| GP11 | — | free | — |
| GP12 | SPI1 MISO | W25Q128 flash (DO) | `PIN_FLASH_MISO` |
| GP13 | SPI1 CS | W25Q128 flash (/CS) | `PIN_FLASH_CS` |
| GP14 | SPI1 SCK | W25Q128 flash (CLK) | `PIN_FLASH_SCK` |
| GP15 | SPI1 MOSI | W25Q128 flash (DI) | `PIN_FLASH_MOSI` |
| GP16 | SPI0 MISO | LoRa SX1278 | `PIN_LORA_MISO` |
| GP17 | SPI0 CS | LoRa NSS | `PIN_LORA_CS` |
| GP18 | SPI0 SCK | LoRa SCK | `PIN_LORA_SCK` |
| GP19 | SPI0 MOSI | LoRa MOSI | `PIN_LORA_MOSI` |
| GP20 | digital in | LoRa DIO0, hardware interrupt | `PIN_LORA_DIO0` |
| GP21 | digital out | LoRa reset | `PIN_LORA_RST` |
| GP22 | digital out | S16 — spare output (17th servo, camera, release) | `PIN_AUX_S16` |
| GP23–25 | — | Pico internal (SMPS, VBUS sense, LED) | — |
| GP26 / ADC0 | analog in | ACS712 current sensor | `PIN_I_SENSE` |
| GP27 / ADC1 | analog in | 2S battery voltage through a 3:1 divider | `PIN_V_BAT` |
| GP28 / ADC2 | analog / digital in | RBF pin, arming interlock | `PIN_RBF` |

### PCA9685 servo channels

The PCA9685 hangs off GP8/GP9. Its outputs are not MCU pins, but they are part of
the pinout in every sense that matters on a launch day.

| channel | actuator | constant |
|---|---|---|
| 0 | TVC servo, body axis 1 | `CH_TVC_X` |
| 1 | TVC servo, body axis 2 | `CH_TVC_Y` |
| 2 | thrust clamp | `CH_CLAMP` |
| 3–15 | unused | — |

> **Digital servos only.** The PWM frequency is global across all sixteen channels
> and is set to 339 Hz to keep servo transport lag inside the 200 Hz control loop's
> budget. Most analogue servos will buzz or overheat at that rate.

---

## 2. Position MCU — Seeed XIAO RP2350

Pin names are the XIAO silkscreen aliases (`D0`…`D12`) that the arduino-pico board
variant defines. The spreadsheet's second column names the *pin's function*; its
third names *what is physically on it*. Where those two disagree — D0 and D1 are
labelled "UART0" but carry the DWM1000's interrupt and reset — the third column
wins, and the firmware follows the third column.

| pin | function | peripheral | constant |
|---|---|---|---|
| D0 (A0) | digital in | **UWB DWM1000 IRQ** | `PIN_UWB_INT` |
| D1 (A1) | open-drain out | **UWB DWM1000 RST** | `PIN_UWB_RST` |
| D2 (A2) | digital out | **UWB DWM1000 CS** | `PIN_UWB_CS` |
| D3 (A3) | digital out | Voltino TriSense Pro CS (ICM42688P) | `PIN_TRISENSE_CS` |
| D4 (A4) | I2C SDA | BMP580, AK09918C, TF Luna lidar | `PIN_I2C_SDA` |
| D5 (A5) | I2C SCL | same bus | `PIN_I2C_SCL` |
| D6 (A6) | UART TX | Control MCU link, 921600 baud | `PIN_MASTER_TX` |
| D7 (A7) | UART RX | Control MCU link | `PIN_MASTER_RX` |
| D8 (A8) | SPI SCK | shared: DWM1000 **and** TriSense | `PIN_SPI_SCK` |
| D9 (A9) | SPI MISO | shared | `PIN_SPI_MISO` |
| D10 (A10) | SPI MOSI | shared | `PIN_SPI_MOSI` |
| D11 | PIO UART TX | LK76 GPS module (to the module's RX) | `PIN_GPS_TX` |
| D12 | PIO UART RX | LK76 GPS module (from the module's TX) | `PIN_GPS_RX` |
| D13–D15 | — | free | — |
| on-board | PIO | WS2812 navigation annunciator | `ARIADNA_RGB_PIN` |

**Three notes that will cost you a bench session if you skip them.**

1. **The SPI bus is shared** between the DWM1000 and the ICM42688P. Both drivers
   use per-transaction `SPISettings`, so the clock and mode are re-established for
   every transfer and neither can inherit the other's. Both are driven from **core 0
   only**, which is what makes the bus safe without a lock.
2. **The DWM1000's reset is open-drain.** It is pulled LOW to reset and then
   *released to a floating input*. Driving it high can damage the module. The
   ranging library is told the reset pin is `255` so that it never drives it either.
3. **D11/D12 are not a hardware UART.** The GPS runs on a PIO state machine, which
   the RP2350 will put on any pin. If the module is silent, swap the two — the
   names describe the RP2350 end.

---

## 3. Navigation (Guidance) MCU — Seeed XIAO RP2350

| pin | function | peripheral | constant |
|---|---|---|---|
| USB-C | USB CDC | pad console / pre-launch link to the rail | `Serial` |
| D0 (A0) | analog in | pyro continuity, channel 1 | `PIN_CONT_1` |
| D1 (A1) | analog in | pyro continuity, channel 2 | `PIN_CONT_2` |
| D2 (A2) | analog in | pyro continuity, channel 3 | `PIN_CONT_3` |
| D3 (A3) | digital out | **pyro channel 1 — landing motor igniter** | `PIN_PYRO_1` |
| D4 (A4) | I2C SDA | broken out, unused by this firmware | — |
| D5 (A5) | I2C SCL | broken out, unused | — |
| D6 (A6) | UART TX | Control MCU link, 921600 baud | `PIN_MASTER_TX` |
| D7 (A7) | UART RX | Control MCU link | `PIN_MASTER_RX` |
| D8 (A8) | digital out | **pyro channel 2 — backup igniter, same motor** | `PIN_PYRO_2` |
| D9 (A9) | digital out | **pyro channel 3 — reserve, never auto-fired** | `PIN_PYRO_3` |
| D10 (A10) | — | reserved for future use | — |
| on-board | PIO | WS2812 pyro/guidance annunciator | `ARIADNA_RGB_PIN` |

> The continuity sense polarity and threshold (`CONT_ACTIVE_HIGH`,
> `CONT_THRESHOLD`) are assumptions about *your* harness. Measure the ADC counts
> with a known-good igniter and with an open channel, and put the midpoint in the
> sketch. Getting the polarity backwards means the board reports a good igniter on
> an empty channel.

---

## 4. Ground station — Raspberry Pi Pico 2

| pin | function | peripheral | constant |
|---|---|---|---|
| GP3 | digital in, active LOW | TEST-mode ON button | `TEST_ON_PIN` |
| GP4 | I2C SDA | SSD1306 OLED | `I2C_SDA` |
| GP5 | I2C SCL | SSD1306 OLED | `I2C_SCL` |
| GP6 | digital in, active LOW | TEST-mode OFF button | `TEST_OFF_PIN` |
| GP10 | digital in, active HIGH | reset button | `RESET_BTN_PIN` |
| GP11 | SPI1 CS | LoRa NSS | `LORA_CS` |
| GP12 | SPI1 MISO | LoRa | `LORA_MISO` |
| GP14 | SPI1 SCK | LoRa | `LORA_SCK` |
| GP15 | SPI1 MOSI | LoRa | `LORA_MOSI` |
| GP20 | digital out | LoRa reset | `LORA_RESET` |
| GP21 | digital in | LoRa DIO0 | `LORA_DIO0` |
| USB | USB CDC | serial link to the visualiser laptop | `Serial` |

---

## 5. UWB anchor ×4 — Seeed XIAO RP2350

Four identical boards, one sketch, differing only in `ANCHOR_ID`.

| pin | function | peripheral | constant |
|---|---|---|---|
| D0 (A0) | digital in | DWM1000 IRQ | `PIN_UWB_INT` |
| D1 (A1) | open-drain out | DWM1000 RST | `PIN_UWB_RST` |
| D2 (A2) | digital out | DWM1000 CS | `PIN_UWB_CS` |
| D8 (A8) | SPI SCK | DWM1000 | `PIN_SPI_SCK` |
| D9 (A9) | SPI MISO | DWM1000 | `PIN_SPI_MISO` |
| D10 (A10) | SPI MOSI | DWM1000 | `PIN_SPI_MOSI` |
| USB-C | USB CDC | console, ranging log | `Serial` |
| on-board | PIO | WS2812 anchor-health annunciator | `ARIADNA_RGB_PIN` |

### Anchor identity

| ANCHOR_ID | EUI | short address | table row in `position_mcu.ino` |
|---|---|---|---|
| 1 | `01:00:5B:D5:A9:9A:E2:9C` | `0x0001` | `s_uwb_anchors[0]` |
| 2 | `02:00:5B:D5:A9:9A:E2:9C` | `0x0002` | `s_uwb_anchors[1]` |
| 3 | `03:00:5B:D5:A9:9A:E2:9C` | `0x0003` | `s_uwb_anchors[2]` |
| 4 | `04:00:5B:D5:A9:9A:E2:9C` | `0x0004` | `s_uwb_anchors[3]` |

The tag on the vehicle uses EUI `AA:00:5B:D5:A9:9A:E2:9C`, short address `0x00AA`.

> `startAsAnchor()` **must** be called with `false` as its third argument. The
> library otherwise randomises the short address at every power-on, the tag matches
> no row of the anchor table, and every range is silently discarded — the radios
> work, the LEDs blink, and the navigation filter never sees a UWB fix.

---

## 6. Inter-board wiring

```
   Position MCU (XIAO)          Control MCU (Pico 2 W)         Navigation MCU (XIAO)
        D6  TX  ────────────────►  GP1  RX
        D7  RX  ◄────────────────  GP0  TX
                                   GP4  TX  ──────────────────►  D7  RX
                                   GP5  RX  ◄──────────────────  D6  TX
        RUN     ◄────────────────  GP6  (reset)
                                   GP7  (reset)  ──────────────►  RUN
```

All three inter-processor links run at **921600 baud**, 8N1, framed by the codec in
`common/ariadna_link.h`. Grounds must be common; the reset lines are active LOW.

---

## 7. Power and interlocks

| item | where | note |
|---|---|---|
| 2S LiPo | Control MCU GP27 via a 3:1 divider | `VBAT_DIVIDER` in `master_mcu.ino` |
| current | Control MCU GP26, ACS712 | 5 V part, 100 mV/A, 2:1 divider — zero at 1.252 V on the pin |
| RBF | Control MCU GP28, pull-up | tag fitted = pin grounded = pyros inhibited |
| breakwire | Control MCU GP10, pull-up | broken = launch detected |
| pyro arming | Navigation MCU, `pyro_permitted()` | requires the Master's armed state |

---

## 8. Annunciator quick reference

Three LEDs, three different questions. See the annunciator section in each sketch
for the full table.

| board | LED | answers |
|---|---|---|
| Control | SK6812 on GP2 + buzzer on GP3 | *what is the mission doing?* |
| Position | XIAO on-board WS2812 | *does the vehicle know where it is?* |
| Navigation | XIAO on-board WS2812 | *is it ready to fire a pyro?* |
| Anchor | XIAO on-board WS2812 | *is this anchor ranging?* |

The one to wait for before arming is the **Position MCU's solid green**: it means a
pad reference is captured and UWB fixes are being *accepted* by the navigation
filter, not merely produced by the radio.

The one to act on is the Position MCU's **yellow, three flashes**: the barometer trim
has hit its clamp, meaning UWB and the pad reference disagree about where the ground
is by more than a barometer plausibly drifts. That is an anchor survey error. It is
fixable in the ten minutes before launch and not fixable afterwards.
