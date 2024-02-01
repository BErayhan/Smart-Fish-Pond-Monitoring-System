#include <UrlEncode.h>
#include <HTTPClient.h>
#include "GravityTDS.h"
#include <LiquidCrystal_I2C.h> //memanggil library LCD
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <EEPROM.h>
#include <esp_sleep.h>
#include "ThingSpeak.h"
#include <WiFiManager.h> 
#include <FS.h> 
#include <SPIFFS.h> 
#include <ArduinoJson.h>
 



#define TdsSensorPin 35 
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30  
#define TDS_THRESHOLD1 60 // Ambang batas TDS
#define TDS_THRESHOLD2 70 // Ambang batas TDS
#define LCD_ADDRESS 0x27 // Alamat LCD I2C
#define LCD_COLUMNS 16
#define LCD_ROWS 2

const int relay = 33; //pin relay (pompa)
const int ph_pin= 34 ; //pin sensor ph
//const int WaktuTidur =  ; //lama deepsleep 60 menit 

const int myChannelNumber = 2371215 ;
const char* apiKey1 = "YGZQPAM38A546YV4";
const char* server = "api.thingspeak.com";

//buat whatsapp
String phoneNumber = "6289604415126";
String apiKey = "1465063";

 
WiFiClient client;


LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS); // Inisialisasi LCD
GravityTDS gravityTds;

//kalibrasi tds
int analogBuffer[SCOUNT];     
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
int flag = 0;


float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;      

//kalibrasi sensor ph
float Po = 0;
float PH_step ;
int nilai_analog_PH;
double teganganPH;
float ph4 = 3.1;
float ph7 = 2.6;

//perhitungan tds : median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void setup() {
  Serial.begin(115200);

  // Inisialisasi LCD
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Kelompok 14");
  
  lcd.setCursor(0, 1);
  lcd.print("Hubungkan Wifi !");
 
  
  //inisialisasi sensor tds
  pinMode(TdsSensorPin, INPUT);
  
  //inisialisasi sensor ph
   pinMode(ph_pin,INPUT);

   //inisialisasi relay
   pinMode(relay, OUTPUT);
   digitalWrite(relay, HIGH);

   esp_sleep_enable_timer_wakeup(36000000); // dalam mikrosekon
   delay(2000);

   //thingspeak + wifi manager
   WiFi.mode(WIFI_STA);
   WiFiManager wm;
  // wm.resetSettings();
   bool res;
   res = wm.autoConnect("Kelompok14_Despro","password"); // password protected ap
    if(!res) {
        Serial.println("Failed to connect");
            lcd.clear();
            lcd.setCursor(5, 0);
            lcd.print("GAGAL"); 
            lcd.setCursor(3, 1);
            lcd.print("TERHUBUNG");  
            delay(2000);
           
         ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
            lcd.clear();
            lcd.setCursor(3, 0);
            lcd.print("BERHASIL"); 
            lcd.setCursor(3, 1);
            lcd.print("TERHUBUNG"); 
            delay(2000);
            
    }
   lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TDS: ");
  lcd.setCursor(0, 1);
  lcd.print("PH: ");
  ThingSpeak.begin(client);

  while(true){
    digitalWrite(relay, HIGH);

    // Sensor PH
    nilai_analog_PH = analogRead(ph_pin);
    Serial.print("Nilai ADC Ph: ");
    Serial.println(nilai_analog_PH);
    teganganPH = 9 / 1024.0 * nilai_analog_PH;
    Serial.print("teganganPH: ");
    Serial.println(teganganPH, 3);

    PH_step = (ph4 - ph7) / 3;
    Po = 7.00 + ((ph7 - teganganPH) / PH_step);
    lcd.setCursor(5, 1);
    lcd.print(Po);

    // Sensor TDS
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U) {
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
      analogBufferIndex++;
      if (analogBufferIndex == SCOUNT) {
        analogBufferIndex = 0;
      }
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U) {
      printTimepoint = millis();
      for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
        float compensationVoltage = averageVoltage / compensationCoefficient;
        tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
      }
    }

    lcd.setCursor(5, 0);
    lcd.print(tdsValue);
    lcd.print(" ppm");

    flag++;
    if (flag == 10 * 60) {
        lcd.clear();
        lcd.setCursor(2, 0);
        lcd.print("DEEPSLEEP ON");
      esp_deep_sleep_start();
      flag = 0;
    }

    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Serial.println(" ppm");

    Serial.print("Nilai PH cairan: ");
    Serial.println(Po, 2);

    // Thingspeak
    static int counter = 0;
    counter++;
    if (counter == 60) {
      ThingSpeak.setField(1, tdsValue);
      ThingSpeak.setField(2, Po);

      int x = ThingSpeak.writeFields(1, apiKey1);

      if (x == 200) {
        Serial.println("Channel update successful.");
      } else {
        Serial.println("Problem updating channel. HTTP error code " + String(x));
      }
      delay(15000);

      counter = 0;

      if (TDS_THRESHOLD1 < tdsValue && tdsValue < TDS_THRESHOLD2) {
        digitalWrite(relay, LOW);
        delay(10000);
        digitalWrite(relay, HIGH);

        lcd.clear();
        lcd.setCursor(2, 0);
        lcd.print("AIR KOTOR ! ");
        delay(4000);
       
        lcd.clear();
        lcd.setCursor(2, 0);
        lcd.print("DEEPSLEEP ON");
        sendMessage("AIR KOLAM DALAM KEADAAN KOTOR");
        esp_deep_sleep_start();
        esp_sleep_enable_timer_wakeup(60000000); // 60 detik
      } else {
        sendMessage("AIR KOLAM DALAM KEADAAN BERSIH");
        digitalWrite(relay, HIGH);
      }
    }

    delay(1000);
  }
}

void loop() {
}

void sendMessage(String message){

  // Data to send with HTTP POST
  String url = "https://api.callmebot.com/whatsapp.php?phone=" + phoneNumber + "&apikey=" + apiKey + "&text=" + urlEncode(message);    
  HTTPClient http;
  http.begin(url);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  // Send HTTP POST request
  int httpResponseCode = http.POST(url);
  if (httpResponseCode == 200){
    Serial.print("Message sent successfully");
  }
  else{ 
    Serial.println("Error sending the message");
    Serial.print("HTTP response code: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}
