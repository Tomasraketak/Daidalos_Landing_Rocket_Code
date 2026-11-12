# Examples — Ariadna V2

Small standalone sketches for trying out one piece of the hardware. Like the bench
tests, each replaces the flight firmware on its board and is flashed back over
afterwards. Console: **115200 baud**, line ending *Newline*.

| sketch | board | libraries | what it does |
|---|---|---|---|
| `Servo_RGB_Demo/Servo_RGB_Demo.ino` | Master — Raspberry Pi Pico 2 W | Adafruit_NeoPixel | servos 1–8 (PCA9685 ch 0–7) to a typed angle, or oscillating between two typed angles: fast, 5 s, fast back, 3 s, slow at 5 °/s, 3 s, slow back, 5 s, repeat; the RGB LED shows what the servos are doing |

## Servo_RGB_Demo

| type | effect |
|---|---|
| `90` | all servos to 90° and hold |
| `30 120` (or `30-120`, `30,120`) | oscillate between 30° and 120° until stopped |
| `s` | stop, hold where they are |
| `c` | centre, 90° |
| `h` | help |

90° = 1500 µs = neutral, 10 µs per degree, limited to 30–150° (900–2100 µs, the flight
code's travel guard). **The TVC and clamp linkages have much less travel than a bare
servo: unhook them, or narrow `ANGLE_MIN` / `ANGLE_MAX`, before commanding large angles.**
PWM is 50 Hz so any servo works; the flight rate of 339 Hz is for digital servos only.

| LED | servos |
|---|---|
| slow blue blink | waiting for the first command |
| blue → green → red, dim | holding; the colour is the angle (30° blue, 90° green, 150° red) |
| red, fast flashing | fast move |
| blue → green → red, bright, drifting | slow move; the colour follows the servo |
| green breathing | waiting between moves |
| yellow blinking | the next move starts within a second |
| dim white | stopped with `s` |
| red / blue alternating | no PCA9685 on the bus |
