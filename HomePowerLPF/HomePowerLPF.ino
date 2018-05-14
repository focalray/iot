/*
 * HOME POWER METER
 * FOR ESP8266-01 (WITH FLASH OF 1MB)
 * SUPPORTS OTA UPDATE
 * LAST UPDATE: 2018-04-30
*/

#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ESP8266WiFi.h>
#include "ThingSpeak.h"
#include "EEPROM.h"
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// The Indicator LED(Blue) and TX cannot be used at the same time!
// Uncomment the following to use the indicator.
#define USEINDICATOR

Adafruit_ADS1115 ads(0x48);  /* Use this for the 16-bit version */
static unsigned long myChannelNumber = xxxxxx;
static const char * myWriteAPIKey = "xxxxxxxxx";
static const char* mqtt_server = "m11.cloudmqtt.com";
static const char* TOPIC = "/iot/ha/homepower";
WiFiClient  clientThingSpeak;
static WiFiClient clientMQTT;
static PubSubClient mqtt(clientMQTT);
bool bUpdatingNow = false;

const int samplingTime = 1000000;             // Measurement duration (Microseconds)
unsigned long previousMillis = 0;
unsigned long rebootMillis = 0;
unsigned long reportMillis = 0;
float Ak = 0.0;
//const float alpha = 0.9;

// Kalman Filter Variable Definitions
static float kf_A;
static float kf_H;
static float kf_Q;
static float kf_R;
static float kf_x;
static float kf_P;

void setup()
{
  Serial.begin(9600);
  Serial.println("\nHome Power Meter V1.3 by RAYEVER (bitspree@gmail.com)");
  Serial.print("\nFlash Size(bytes) = "); Serial.println(ESP.getFlashChipRealSize());
  Serial.println("Flash Size under 1MB cannot be updated via OTA.");

  init_network();

  mqtt.setServer(mqtt_server, 11140);
  mqtt.setCallback(mqtt_callback);

  // Initialize ThingSpeak
  ThingSpeak.begin(clientThingSpeak);

  // Initialize high-res ADC
  Wire.begin(0, 2);
  ads.begin();

  // Initialize Estimation System
  //  stabilization = -10;
  Ak = -1.0;
  SimpleKalman_init();
  RestoreAllVariables();

  rebootMillis = millis();
  reportMillis = millis();

  // OTA setup
  ArduinoOTA.setHostname("HomePower");
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setPassword((const char *)"5357");
  ArduinoOTA.onStart([]()
  {
    bUpdatingNow = true;
    indicator(10, 100);
  });

  ArduinoOTA.onEnd([]()
  {
    bUpdatingNow = false;
    indicator(10, 100);
  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    (void)error;
    ESP.restart();
  });

  ArduinoOTA.begin();

  indicator(10, 300);
}

void loop()
{
  ArduinoOTA.handle();  
  mqtt_reconnect();

  //////////////////////// CHECK POWER CONSUMPTION ///////////////////////
  unsigned long prevTime = micros();

  int16_t adc0 = 0;
  unsigned long nSample = 0;
  float sumData = 0.0;
  float Vmax = 0;

  while (micros() - prevTime < samplingTime)
  {
    adc0 = ads.readADC_SingleEnded(0);
    float Voltage = (adc0 * 0.1875) / 1000; // 0.1875 = +-6.144V / 32767
    sumData += Voltage;
    nSample++;

    if (Vmax < Voltage)
      Vmax = Voltage;

    delayMicroseconds(1000);  // Wait for noise reduction
  }

  float Vmean = ((float)sumData / nSample);
  float Vpk = Vmax - Vmean;       // automatic calibration
  float A = Vpk / 39.0 * 2000.0;  // 39ohm (burden resistor)

  // Low Pass Filter of 'A'
  //  if(Ak < 0.0)
  //    Ak = A;
  //  else
  //    Ak = alpha*Ak + (1.0-alpha)*A;  // Low Pass Filter

  // Kalman Filter
  SimpleKalman(A, &Ak);

  float P = 220.0 * Ak;

  // 39ohm results in +-2.757V at max current (0.0707A ==> 141.4A / 2000 turns)

  Serial.print("Vmean/Vpk= "); Serial.print(Vmean, 7); Serial.print("/"); Serial.print(Vpk, 7);
  Serial.print(",   I= "); Serial.print(A, 7);
  Serial.print(",   Ie= "); Serial.print(Ak, 7);
  Serial.print(",   P= "); Serial.println(P, 7);

  mqtt.publish(TOPIC, String(P).c_str());

  // Report to ThingSpeak every 20 seconds.
  unsigned long currentMillis = millis();
  if (currentMillis - reportMillis >= (1000 * 20))
  {
    ThingSpeak.setField(1, Ak);
    ThingSpeak.setField(2, P);
    ThingSpeak.setField(3, A);
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    reportMillis = currentMillis;
    mqtt.publish(TOPIC, "Post");
  }

  // Restart every 10 minutes...
  currentMillis = millis();
  if (!bUpdatingNow && (currentMillis - rebootMillis >= (1000 * 60 * 10)))
  {
    mqtt.publish(TOPIC, "Restart");
    // I've been experienced that the system is halted unexpectedly for unknown reason.
    SaveAllVariables();
    Serial.println("Reboot...");
    indicator(20, 100);
    ESP.restart();
  }

  // This makes a delay too.
  indicator(2, 200);

  mqtt.loop();
}

void indicator(int n, int d)
{
#ifdef USEINDICATOR
  pinMode(LED_BUILTIN, OUTPUT);
  for (int k = 0; k < n; k++)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(d);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(d);
    yield();
  }
#endif
}

void init_network()
{
  // Connect to wifi
  WiFi.begin("olleh_WiFi_E160", "0000008608");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    indicator(1, 300);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//
// Arguments    : float z
//                float *volt
// Return Type  : void
void SimpleKalman(float z, float *volt)
{
  float xp;
  float Pp;
  float K;
  xp = kf_A * kf_x;
  Pp = kf_A * kf_P * kf_A + kf_Q;
  K = Pp * kf_H * (1.0 / (kf_H * Pp * kf_H + kf_R));
  kf_x = xp + K * (z - kf_H * xp);
  kf_P = Pp - K * kf_H * Pp;
  *volt = kf_x;
}

void SimpleKalman_init()
{
  kf_A = 1.0;
  kf_H = 1.0;
  kf_Q = 0.0;
  kf_R = 1.5;
  kf_x = 2.0;
  kf_P = 3.0;
}

void SaveAllVariables()
{
  EEPROM.begin(512);
  EEPROM.put<float>(0, kf_x);
  EEPROM.put<float>(4, kf_P);
  EEPROM.commit();
}

void RestoreAllVariables()
{
  EEPROM.begin(512);
  EEPROM.get<float>(0, kf_x);
  EEPROM.get<float>(4, kf_P);
}

/********************************************************************************************************
   MQTT
*/
void mqtt_callback(char* topic, byte * payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  if (length > 0)
  {
    String msg;
    for (int i = 0; i < length; i++)
      msg += (char)payload[i];
    Serial.println(msg);

    bool cmd_ok = false;
    if (String(topic).equals(TOPIC))
    {
      if (msg.equals("init"))
      {
        cmd_ok = true;
        indicator(10, 100);
        SimpleKalman_init();
        SaveAllVariables();
        while (1)
        {
          mqtt.publish(TOPIC, "Factory");
          Serial.println("System initialized. It will be rebooted after 5 seconds...");
          delay(5000);
          indicator(20, 100);
          ESP.restart();
        }
      }
    }
  }
  else
  {
    Serial.println();
  }
}

void mqtt_reconnect()
{
  // Loop until we're reconnected
  while (!mqtt.connected())
  {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqtt.connect("remote_iot", "xxxxxxx", "xxxxxxxx"))
    {
      Serial.println("connected");
      mqtt.publish(TOPIC, "Connected");
      mqtt.subscribe(TOPIC);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


