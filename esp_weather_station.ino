#include "DHT.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht;
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C
#include <EEPROM.h>

#include <ESP8266WiFi.h>
String WIFI_Hostname;
WiFiClient client;
#include <SimplePortal.h>
#define BUTTON_PIN       4   // The ESP8266 pin connected to button
#define LONG_PRESS_TIME  3000 // 1000 milliseconds
#define SHORT_PRESS_TIME  100 // 1000 milliseconds
int prev_button_state = HIGH;  // The previous state from the input pin
int prev_short_state = HIGH;
int button_state;     // The current reading from the input pin
int short_state;     // The current reading from the input pin
unsigned long time_pressed  = 0;
unsigned long short_pressed  = 0;
unsigned long time_released = 0;
unsigned long short_released = 0;
struct Cfg {
  char ssid[32];
  char pass[32];
};
// глобальный экземпляр для личного использования
Cfg cfg;
#define STR_ADDR 0
#define EEPROM_SIZE 64
String ssid;
String pass;
String narodmonstatus = "UNKNOWN";

#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// настройки температурного сенсора
#define pinDHT 5 // Sensor Pin
DHT dht(pinDHT, DHT22);

// настройки OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32, меняем поскольку китайцы гады, для нестандарта.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool display_enabled = true;

// переменные для передачи и вывода на экран
float temp,inside,hum,press,lux,humin;

void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

void setup()
{
  Serial.begin(115200);
  dht.begin(); //init temp sensor

  // инициализируем OLED дисплей
  Wire.begin(14, 12); //SDA, SCL меняем поскольку дисплей приварен на эти пины по I2C китайцы гады еще те
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
    // Clear the buffer
  display.clearDisplay();
  display.drawPixel(10, 10, SSD1306_WHITE);
  display.display();

  //try AHT sensor
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

// setup BMP sensor
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  /* Initialise the sensor */
  //use tsl.begin() to default to Wire, 
  //tsl.begin(&Wire2) directs api to use Wire2, etc.
  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    /* Setup the sensor gain and integration time */
  configureSensor();
    // инициализируем кнопку
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button_state = digitalRead(BUTTON_PIN);
  short_state = digitalRead(BUTTON_PIN);

  // настраиваем WiFi
   //Init EEPROM
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(STR_ADDR, cfg);
  ssid = cfg.ssid;
  pass = cfg.pass;
  Serial.print("ssid: ");
  Serial.println(ssid);
  Serial.print("pass: ");
  Serial.println(pass);
  if ((ssid == "" && pass == "") || (button_state == LOW )) {
    setupwifi();
  }
  connectwifi();
}

void setupwifi() {
// captive portal setup
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);             // Start at top-left corner
  display.print("Running Wifi portal");
  display.display();

  Serial.println("setup wifi");
  portalRun(120000);
  Serial.println(portalStatus());
  if (portalStatus() == SP_SUBMIT) {
    Serial.println(portalCfg.SSID);
    Serial.println(portalCfg.pass);
    strcpy(cfg.ssid, portalCfg.SSID);
    strcpy(cfg.pass, portalCfg.pass);
    EEPROM.put(STR_ADDR, cfg);
    EEPROM.commit();
    ssid = portalCfg.SSID;
    pass = portalCfg.pass;
  }
  else {
    ESP.restart();
  }
}
void connectwifi() {
   // подключаем  wifi
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);             // Start at top-left corner
  display.println("Connecting to WiFi");
  display.print("ssid: ");
  display.println(ssid);
  display.print("pass: ");
  display.println(pass); 
  display.display();

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  int cd = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    if (cd > 120) {
      Serial.println("WiFi problem with connection for 120 sec, go to safe mode");
      setupwifi();
      return;
    }
    cd = cd + 1;
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  WIFI_Hostname = "ESP" + WiFi.macAddress();
  WIFI_Hostname.replace(":", "");
  Serial.print("Wifi_Hostname: ");
  Serial.println(WIFI_Hostname);
}

bool checkbutton() {
    button_state = digitalRead(BUTTON_PIN);
    //Serial.println(button_state);
  if(prev_button_state == HIGH && button_state == LOW) {       // button is pressed
    time_pressed = millis();
  } else if(prev_button_state == LOW && button_state == HIGH) { // button is released
    time_released = millis();
    long press_duration = time_released - time_pressed;

    if( press_duration > LONG_PRESS_TIME ) {
      //Serial.println(time_pressed);
      //Serial.println(time_released);
      Serial.println("A long press is detected");
      prev_button_state = button_state;
      return true;
    }
  }

  // save the the last state
  prev_button_state = button_state;
  return false;
}

bool shortbutton() {
    short_state = digitalRead(BUTTON_PIN);
    //Serial.println(button_state);
  if(prev_short_state == HIGH && short_state == LOW) {       // button is pressed
    short_pressed = millis();
  } else if(prev_short_state == LOW && short_state == HIGH) { // button is released
    short_released = millis();
    long short_duration = short_released - short_pressed;

    if( short_duration > SHORT_PRESS_TIME ) {
      Serial.println("A short press is detected");
      prev_short_state = short_state;
      return true;
    }
  }

  // save the the last state
  prev_short_state = short_state;
  return false;
}

void SendToNarodmon(float temp, float pres, float humi, float lux, float inside, float humin)
{
  String buf;
  buf = "#" + WIFI_Hostname + "#esp_meteo" + "\r\n"; // HEADER AND NAME
  buf = buf + "#T1#" + String(temp) + "#ТемператураУл\r\n";
  buf = buf + "#H1#" + String(humi) + "#ВлажностьУл\r\n";
  buf = buf + "#BMPP1#" + String(pres) + "#ДавлениеВн\r\n";
  buf = buf + "#L1#" + String(lux) + "#ОсвещенностьВн\r\n";
  buf = buf + "#T2#" + String(inside) + "#ТемператураВн\r\n";
  buf = buf + "#H2#" + String(humin) + "#ВлажностьВн\r\n";
  //buf = buf + "#LAT#42.085826\r\n";
  //buf = buf + "#LON#19.11504\r\n";
  //buf = buf + "#ALT#13\r\n";
  buf = buf + "##"; // close packet

  if (!client.connect("narodmon.ru", 8283))
  {
    Serial.println("Connect to NarodMon - FAILED");
    narodmonstatus = "FAILED";
  }
  else
  {
    Serial.println("Connect to NarodMon - OK");
    narodmonstatus = "GOOD";
    Serial.println(WIFI_Hostname);
    Serial.println("////////////////////////////////////////////");
    client.print(buf); //send data
    while (client.available())
    {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
  }
}

void read_sensors() {
  temp = dht.readTemperature();
  hum = dht.readHumidity();
  if (isnan(temp) || isnan(hum))
  {
    Serial.println("Определяем сенсоры");
    delay(800);
    return;
  }
  
  Serial.println("Данные с AM2120");
  Serial.print("Температура: ");
  Serial.print(temp);
  Serial.print(" C, ");
  Serial.print("Влажность: ");
  Serial.print(hum);
  Serial.println("% RH.");

  Serial.println("data from AHT20:");
  sensors_event_t hum2, temp2;
  aht.getEvent(&hum2, &temp2);// populate temp and humidity objects with fresh data
  Serial.print("Temperature: "); Serial.print(temp2.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(hum2.relative_humidity); Serial.println("% rH");
  inside = temp2.temperature;
  humin = hum2.relative_humidity;

  Serial.println("data from BMP280:");
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  press = bmp.readPressure()/100;
  Serial.print(press,0);
  Serial.println(" hPa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");
  Serial.println();

  //light sensor TLS2561
    /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    lux = event.light;
    Serial.print(event.light, 0); Serial.println(" lux");
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
    lux = 0;
  }

  Serial.printf("Connection status: %d\n", WiFi.status());
}

void refresh_oled() {
  switch (display_enabled) {
    case 1: {
      String wifistatus;
      // печатаем на дисплее
      display.clearDisplay();
      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);        // Draw white text
      display.setCursor(0,0);             // Start at top-left corner
      display.print("Temp: ");
      display.println(temp);
      display.print("Hum: ");
      display.println(hum);
      display.print("Press: ");
      display.println(press, 0);
      display.print("Lux: ");
      display.println(lux, 0);
      display.print("Wifi: ");
      switch (WiFi.status()) {
        case 3:
          wifistatus = "CONNECTED";
          break;
        case 7:
          wifistatus = "DISCONNECTED";
          break;
        default:
          wifistatus = "OTHER";
          break;
      }
      display.println(wifistatus);
      display.print("T Inside: ");
      display.println(inside);
      display.print("H Inside: ");
      display.println(humin);
      display.print("narodmon: ");
      display.println(narodmonstatus);
      display.display();
      }
      break;
    case 0: {
      display.clearDisplay();
      display.display();
      }
      break;
  }
}

void loop() {  
  static uint32_t tmr1;
  if (millis() - tmr1 >= 300000) {
    tmr1 = millis();
    SendToNarodmon(temp, press, hum, lux, inside, humin);
  }
  
  static uint32_t tmr2;
  if (millis() - tmr2 >= 5000) {
    tmr2 = millis();
    read_sensors();
  }

  static uint32_t tmr3;
  if (millis() - tmr3 >= 1000) {
    tmr3 = millis();
    refresh_oled();
  }
  //check button pressed
  if (checkbutton() == true)
    setupwifi();
  if (shortbutton() == true and checkbutton() == false) {
    Serial.printf("Short press button\n");
    display_enabled = !display_enabled;
  }
  //delay(1000);
}