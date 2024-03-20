#include <WiFiClientSecure.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_SHT4x.h"
#include "SparkFun_SGP30_Arduino_Library.h" 
#include "GasBreakout.h"



#if defined(ADAFRUIT_FEATHER_ESP32_V2)
#define PIN_NEOPIXEL 0
#define NEOPIXEL_I2C_POWER 2
#endif


#if defined(PIN_NEOPIXEL)
  Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

typedef struct {
  float temperature; /**< temperature in degree celcius */
  float relative_humidity; /**< relative humidity in percent */
  double vgas;/**< air quality in percent */
} aaktp_sensors;

typedef struct{
  String ssid;  
  String pass;  
} wifi_router;

typedef struct{
  String serialNo;
  String appVersion;
  IPAddress ipAddress;
  bool isConnected;
} dev_info;

typedef struct{
  double v_ref;
  double v_gas_o;
  double v_offset;
  double concentration;
  double M;
  double v_gas;
} H2S_data;

  Adafruit_SHT4x sht4 = Adafruit_SHT4x();
  GasBreakout gas(Wire, 0x19);
  LiquidCrystal_I2C lcd(0x27,20,4);
  aaktp_sensors sensor;
  H2S_data h2s_data;
  SGP30 mySensor;

  dev_info device_info;
  dev_info * ptr_dev_info = &device_info;

  wifi_router wr[10];
  


void setup() {
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.home();
  lcd.clear();
  Dev_InfosetConnection(false, Dev_Info());

  wr[0].ssid = "Fayolumoses";
  wr[0].pass = "yetrrg88";

  wr[1].ssid = "MTNHyNetflex 2.4Ghz";
  wr[1].pass = "bashiru1";

  setAppVersion(1,0,0, ptr_dev_info);

  StartupPage(Dev_Info()->appVersion);
  delay(3000);
  lcd.clear();
  lcd_progressbar(lcd, "Loading...",0, 20);
  Serial.begin(115200);
  lcd_progressbar(lcd, "Loading...",10, 20);
  Serial.println(F("Initialising Sensors...."));

  Serial.println(F("Initialising SHT sensor"));
  if (! sht4.begin()) {
    Serial.println(F("Couldn't find SHT4x"));
    while (1) delay(1);
  }
  lcd_progressbar(lcd, "Loading...",15, 20);

  Serial.println(F("Found SHT4x sensor"));
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER); 
  lcd_progressbar(lcd, "Loading...",18, 20);

  initialise_H2S_sensor(&h2s_data);
  lcd_progressbar(lcd, "Loading...",30, 20);

  enableInternalPower();
  pixel.begin(); 
  pixel.setBrightness(20); 
  pixel.setPixelColor(0, 0xFF0000);
  pixel.show();
  lcd_progressbar(lcd, "Loading...",40, 20);

  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1);
  }

  lcd_progressbar(lcd, "Loading...",60, 20);
  mySensor.initAirQuality();
  lcd_progressbar(lcd, "Loading...",65, 20);
  if(!gas.initialise()){
      Serial.println("MICS6814 - Not Initialised");
      while(true) {}
  }
  Serial.println("MICS6814 - Initialised");

  for(int i = 0; i < 2; i++){

    for(int retry = 3; retry > 0; retry--){  

      if(!Wifi_Connect((char*)wr[i].ssid.c_str(), (char*)wr[i].pass.c_str(),LED_BUILTIN)){
        Serial.println("Could not connect to Wifi");
        displayWifiConnectFailed(lcd, wr[i].ssid);
        delay(500);
        if (retry > 1) Serial.println("Retrying Network connect");
      }
      else {
        Dev_InfosetConnection(true,Dev_Info());
        break;
      }
      
    }
    if(Dev_Info()->isConnected) break;
    
}
if(!Dev_Info()->isConnected){
  Serial.println("Not connected to Wi-Fi. Restarting System");
  lcd.clear();
  lcd.setCursor(0, 1);

  lcd.print("Wi-Fi not connected");
  lcd.setCursor(0, 2);

  lcd.print("Restarting System");
  delay(2000);
  ESP.restart();
}
  Dev_InfosetIpAddress(WiFi.localIP(), Dev_Info());
  lcd_progressbar(lcd, "Loading...",100, 20);

  printWifiStatus();

}



void loop() {

  read_sht_data(&sensor);
  read_H2S_data(&sensor,h2s_data);
  delay(1000); //Wait 1 second
  //measure CO2 and TVOC levels
  mySensor.measureAirQuality();
  GasBreakout::Reading reading;
  reading = gas.readAll();

  Serial.print("Temperature: "); Serial.print(sensor.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(sensor.relative_humidity); Serial.println("% rH");
  Serial.print("Hydrogen Sulphide: "); Serial.print(sensor.vgas); Serial.println("ppm");
  lcd_progressbar(lcd, "Humidity...",(int)sensor.relative_humidity, 20);
  delay(2000);
  lcd.clear();
  lcd_progressbar(lcd, "Temperature...",(int)sensor.temperature, 20);
  delay(2000);
  lcd.clear();
  lcd_progressbar(lcd, "VGAS...",(int)sensor.vgas, 20);
  delay(2000);
  lcd.clear();

  Serial.println("Reading Gas");
  Serial.print("Red: ");
  Serial.println(reading.reducing);
  Serial.print("NH3: ");
  Serial.println(reading.nh3);
  Serial.print("Ox: ");
  Serial.println(reading.oxidising);
  Serial.println("");
  
  Serial.print("CO2: ");
  Serial.print(mySensor.CO2);
  Serial.print(" ppm\tTVOC: ");
  Serial.print(mySensor.TVOC);
  Serial.println(" ppb");
  
}

int read_sht_data(aaktp_sensors * sensor){
  sensors_event_t humidity, temp;

  if(!sht4.getEvent(&humidity, &temp)){
    Serial.println(F("SHT 40: Error!! Could not get event"));
    return -1;
  }

  sensor->relative_humidity = humidity.relative_humidity;
  sensor->temperature = temp.temperature;

  return 0;
}

int init_sensors(){
  Serial.println(F("Initialising Sensors...."));

  Serial.println(F("Initialising SHT sensor"));
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }

  Serial.println(F("Found SHT4x sensor"));
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);  

}

double readADC(int adc_pin, int max_input, int min_input){
  double ret;

  size_t data = analogRead(adc_pin);
  
  ret = map_float(data, max_input,min_input,3.3, 0.0);

  return ret;
}


int lcd_progressbar(LiquidCrystal_I2C lcd, String Title, uint8_t percent, int width){
  if(percent >100){
    return -1;
  }
  
  String toPrint = Title + "("+ String(percent)+ "%)";
  int len = toPrint.length();

  lcd.setCursor(0,0);
  for(int i = 0; i < width; i++){
    lcd.print('-');
  }
  lcd.setCursor(int((width - len)/2), 1);
  lcd.print(toPrint);

    lcd.setCursor(0,2);
    int _percent = map(percent, 100,0, width, 0);
    for(int i = 0; i < _percent; i++){
      lcd.write(255);
    }
  lcd.setCursor(0,3);
  for(int i = 0; i < width; i++){
    lcd.print('-');
  }


  return 0;
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

bool Wifi_Connect(char * SSID, char * password, int Status_LED){
  bool ret = false;
  pinMode(Status_LED, OUTPUT);
  digitalWrite(Status_LED, LOW);
  Serial.println("Connecting to WiFi:" + String(SSID) + "\tPassword:" + String(password));
  WiFi.begin(SSID, password);
  for(int countdown = 30; countdown > 0; countdown--){ 
    Serial.println("---------------------------------Count Down (" + String(countdown) + ")---------------------------------" );
    if(WiFi.status() != WL_CONNECTED) {
      digitalWrite(Status_LED, HIGH);
      delay(250);
      digitalWrite(Status_LED, LOW);
      delay(250);
      ret = false;
    }
    else{
      Serial.println("Connected to WiFi:" + String(SSID));
      pixel.setPixelColor(0, 0x00FF00);
      pixel.show();
      ret = true;
      break;
    }
  }
  return ret;

}

void displayWifiConnectFailed(LiquidCrystal_I2C lcd, String SSID){
  String width = "Wi-Fi ERROR";
  String text = "Could not connect to";
  lcd.clear();
  lcd.setCursor(int((20 - width.length())/2), 0);
  lcd.print(width);

  lcd.setCursor(int((20 - text.length())/2), 2);
  lcd.print(text);

  lcd.setCursor(int((20 - SSID.length())/2), 3);
  lcd.print(SSID);

}

void enableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  pinMode(PIN_I2C_POWER, INPUT);
  delay(1);
  bool polarity = digitalRead(PIN_I2C_POWER);
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
}

void disableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to rest state (off)
  pinMode(PIN_I2C_POWER, INPUT);
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif
}

void StartupPage(String version){
  String app_ver = "V"+ version;
  lcd.clear();
  lcd.setCursor(5, 1);
  lcd.print("AAKTP Node");
  lcd.setCursor((int)((20- app_ver.length())/2),2);
  lcd.print(app_ver);
}

//8742093156

void setSerialNo(String serial_no, dev_info *d){
  d->serialNo = serial_no;
}

dev_info * Dev_Info(){
  return ptr_dev_info;
}

String getSerialNo(){
  return Dev_Info()->serialNo;
}

void setAppVersion(int a, int b, int c, dev_info *d){
  String temp = String(a) + "." + String(b) + "." + String(c);
  d->appVersion = temp;
}

void read_H2S_data(aaktp_sensors * sensor, H2S_data h2s){
  double data = readADC(32,4095,0);
  Serial.println("H2S: Data:"+ String(data));
  sensor->vgas = (double)((1/h2s.M)*(data - h2s.v_gas_o));

}

void initialise_H2S_sensor(H2S_data * h2s){
  double sensitivity_gain = 4.94;
  double TIA_gain = 49.9;
  Serial.println(F("Initialising H2S Sensor"));
  h2s->v_ref = 3.3/2;
  h2s->v_offset = 0.0;
  h2s->v_gas_o = h2s->v_ref - h2s->v_offset;
  h2s->M = 2.46506 * pow(10,-4);
  h2s->v_gas = readADC(32,15,7);
  h2s->concentration = (double)((1/h2s->M)*(h2s->v_gas - h2s->v_gas_o));

  Serial.println("V_Ref: "+ String (h2s->v_ref));
  Serial.println("V_Gas_o: "+ String (h2s->v_gas_o));
  Serial.println("M: "+ String (h2s->M));
}

double map_float(int value, int from_high, int from_low, double to_high, double to_low){
  double res = ((double)(value - from_low) * (double)(to_high - to_low))/(double)(from_high - from_low);

  return res + to_low;
}

void Dev_InfosetConnection(bool connection, dev_info * d){
  d->isConnected = connection;
}

void Dev_InfosetIpAddress(IPAddress ip, dev_info * d){
  d->ipAddress = ip;
}

IPAddress getDeviceIP(){
  return Dev_Info()->ipAddress;
}

