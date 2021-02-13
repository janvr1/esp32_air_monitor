### TO-DO:
* (SW) Reconnect v primeru, da zmanjka wifi-ja
* (SW) Fix memory leak
* (SW) Ubidots

### Notes:
* GPIO 6-11: SPI flash
* GPIO 16-17: PSRAM
* GPIO 34-39: Input only
* EWMA filter: alpha = 2\*pi\*fc\*Ts/(1+2\*pi\*fc\*Ts)

### Display timings:
* F_pwm = 312,5 kHz (T=3,2 us)
* PWM: T_low@100%duty=~20 ns
* T_transfer = ~22 us (2 rows of 64 pixels * 3 colors)
* T_row = 200 us (T_transfer + 178 delay)
* Minimum brightness: T_on = 22 us (transfer time), T_off = 178 us (178 us delay)
* Maximum brightness: T_on = 200 us (22 us transfer + 178 us delay), T_off = 0 us
* Frame rate: 200..250 Hz (daylight), 100 Hz (nighttime)
