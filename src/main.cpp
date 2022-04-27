#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Credentials.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <HX711.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <string>

// follows the guide on https://honey-pi.de/en/4x-half-bridge-waegezellen/

// status LED
static const int led_pin = 2;

// the scale's ADC
HX711 scale;
static const int scale_dout_pin = 3;
static const int scale_sck_pin = 0;

// mqtt
#define MOSQUITTO_PORT 1883
static const char *mqtt_server = "192.168.0.162";
static const char *mqtt_clientid = "scale";
static const char *mqtt_in_topic = "scale/in";

// wifi client
WiFiClient espClient;

// mqtt client
PubSubClient mqtt_client(espClient);
void mqtt_callback(char *topic, byte *payload, uint16_t length);
void mqtt_connect();
int16_t get_number_in_message(const char *message);
double get_double_in_message(const char *message);

// ticker for timed callbacks, e.g. blinking leds
Ticker ticker;

// status helpers
void blink_red_led();
void signal_setup_complete();

// setup helpers
void setup_hx711();
void setup_serial();
void setup_wifi();
void setup_ota();
void setup_mqtt();

// scale helpers
void scale_read_and_report();
void scale_calibrate(float cal_weight_grams, float tolerance = 20);

// EEPROM helpers - values have hardcoded addresses
static const int16_t scale_calibration_address = 0x00;

void setup()
{
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);

  ticker.attach_ms(100, blink_red_led);

  EEPROM.begin(sizeof(float));
  setup_serial();
  setup_wifi();
  setup_ota();
  setup_mqtt();
  setup_hx711();

  ticker.detach();
  signal_setup_complete();
}

void loop()
{
  ArduinoOTA.handle();

  if (!mqtt_client.connected())
  {
    delay(2000);
    ESP.restart();
  }
  mqtt_client.loop();

  scale_read_and_report();
}

void setup_serial()
{
  Serial.begin(115200);
}

void setup_wifi()
{
  // set up wifi and OTA
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("success");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_ota()
{
  ArduinoOTA.setHostname("scale");
  ArduinoOTA.setPassword("prost");

  ArduinoOTA.begin();
  Serial.println("Arduino OTA ready");
}

void setup_hx711()
{
  scale.begin(scale_dout_pin, scale_sck_pin, 64);

  float calibration;
  EEPROM.get(scale_calibration_address, calibration);
  mqtt_client.publish("scale/out/d", ("Loaded calibration factor = " + std::to_string(calibration)).c_str());
  scale.set_scale(calibration);
  scale.tare();
}

void setup_mqtt()
{
  mqtt_client.setServer(mqtt_server, MOSQUITTO_PORT);
  mqtt_client.setCallback(mqtt_callback);
  mqtt_connect();
}

void mqtt_callback(char *topic, byte *payload, uint16_t length)
{
  char message[length + 1];
  for (uint16_t i = 0; i < length; ++i)
  {
    message[i] = payload[i];
  }
  message[length] = '\0';

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);

  if (strncmp(message, "CALIBRATE", 9) == 0)
  {
    float weight = get_double_in_message(message);
    if (weight > 1)
    {
      scale_calibrate(weight, 0.03 * weight); // 3 % tolerance
    }
    else
      mqtt_client.publish("scale/out/d", "Bad calibration weight");
  }
  else if (strcmp(message, "TARE") == 0)
  {
    scale.tare(3);
  }
  else
  {
    mqtt_client.publish("scale/out/e", "Didn't understand message");
  }
}

void mqtt_connect()
{
  int count = 0;
  // Loop until we're reconnected
  while (!mqtt_client.connected() and count < 10)
  {
    count++;
    Serial.print("Attempting MQTT connection (");
    Serial.print(count);
    Serial.print("/10) ");
    // Attempt to connect
    if (mqtt_client.connect(mqtt_clientid))
    {
      Serial.println("connected");
      mqtt_client.publish("scale/out/status", "scale online");
      // ... and resubscribe
      mqtt_client.subscribe(mqtt_in_topic, 0);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.println(mqtt_client.state());
      delay(5000);
      ESP.restart();
    }
  }
}

void blink_red_led()
{
  bool state = digitalRead(led_pin);
  digitalWrite(led_pin, !state);
  if (state)
  {
    // timer to turn LED back off
    ticker.attach_ms(150, blink_red_led);
  }
  else
  {
    // timer to turn LED on
    ticker.attach_ms(50, blink_red_led);
  }
}

void signal_setup_complete()
{
  // LED remains during normal operation
  digitalWrite(led_pin, HIGH);
}

void scale_read_and_report()
{
  long reading = scale.get_value(4);
  float grams = scale.get_units(4);
  mqtt_client.publish("scale/out/raw", std::to_string(reading).c_str());
  mqtt_client.publish("scale/out/g", std::to_string(grams).c_str());
  mqtt_client.publish("scale/out/kg", std::to_string(grams / 1000.f).c_str());
}

void scale_calibrate(float cal_weight_grams, float tolerance)
{
  float orig_scale = scale.get_scale();

  float units;
  float start_units = scale.get_units(3);

  uint32_t timerStart = millis();
  while (abs((units = scale.get_units(3)) - start_units) < 0.5 * cal_weight_grams)
  {
    mqtt_client.publish("scale/out/d", ("Place calibration weight - " + std::to_string(cal_weight_grams) + " grams").c_str());
    if ((millis() - timerStart) > 10 * 1000)
    {
      mqtt_client.publish("scale/out/e", "Calibration timed out");
      return;
    }
    delay(500);
  }

  uint8_t cal_success = 0;
  float calib;
  float step = 2;
  float value = scale.get_value(3);
  float guess = cal_weight_grams / abs(value);

  for (int i = 1; i <= 50; ++i)
  {
    scale.set_scale(guess);
    units = scale.get_units(i > 4 ? 10 : 4);
    mqtt_client.publish("scale/out/d", ("Optimising | units = " + std::to_string(units) + " | guess = " + std::to_string(guess)).c_str());

    if (abs(units - cal_weight_grams) < tolerance)
    {
      mqtt_client.publish("scale/out/d", "Success");
      cal_success++;
      calib = guess;
      step /= 5.0f;
      if (cal_success > 3)
        break;
    }
    else
    {
      if (i % 4 == 0)
        step /= 2.0f;

      if (cal_weight_grams > units)
        guess = guess < step ? guess / 5.f : guess - step;
      else
        guess += step;
    }
  }

  // store to EEPROM
  if (cal_success)
  {
    EEPROM.put(scale_calibration_address, calib);
    if (EEPROM.commit())
      mqtt_client.publish("scale/out/d", "Stored calibration to EEPROM");
    else
    {
      mqtt_client.publish("scale/out/e", "Saving to EEPROM failed!");
    }
  }
  else
    scale.set_scale(orig_scale);
}

int16_t get_number_in_message(const char *message)
{
  const char *ptr = strchr(message, ' ');
  if (ptr && *(ptr + 1))
  {
    return std::atoi(ptr + 1);
  }
  else
  {
    return 0;
  }
}

double get_double_in_message(const char *message)
{
  const char *ptr = strchr(message, ' ');
  if (ptr && *(ptr + 1))
  {
    return std::atof(ptr + 1);
  }
  else
  {
    return 0.0f;
  }
}
