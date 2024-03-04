simple weather station based on esp8266 chip , board chineese nodemcu clone with wired OLED screen.
controls:
- start with pushed button (or long push and release afrer 3 sec) - enter wifi setup mode, run captive portal with "ESP Config" name.
- short button push - toggle on\off OLED screen.

OLED screen displays information from sensors, wifi connection status, and last narodmon sending state.

because chineese nodemcu wired i2c lines at non-standard pins, in code i modify pins with  Wire.begin(14, 12); //SDA, SCL

data is sent to the narodmon server once every 3 minutes by TCP protocol.
