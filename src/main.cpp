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
static const char *mqtt_clientid = "scale1";
static const char *mqtt_in_topic = "scale1/in";
static const char *mqtt_out_e = "scale1/out/e";
static const char *mqtt_out_d = "scale1/out/d";
static const char *mqtt_out_kg = "scale1/out/kg";

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
float last_grams = 0;
uint8_t nreads = 4;

// EEPROM helpers - values have hardcoded addresses
static const int16_t scale_calibration_address = 0x00;

// report helpers
void report_ip();

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
  if (calibration != calibration)
  {
    mqtt_client.publish(mqtt_out_d, "Loaded calibration factor is NaN - resetting to 1 ");
    calibration = 1;
  }
  else
  {
    mqtt_client.publish(mqtt_out_d, ("Loaded calibration factor = " + std::to_string(calibration)).c_str());
  }
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
      scale_calibrate(weight, 0.01 * weight); // 3 % tolerance
    }
    else
      mqtt_client.publish(mqtt_out_d, "Bad calibration weight");
  }
  else if (strcmp(message, "TARE") == 0)
  {
    scale.tare(3);
  }
  else if (strncmp(message, "NREADS", 6) == 0)
  {
    nreads = get_number_in_message(message);
    mqtt_client.publish(mqtt_out_d, ("New nreads = " + std::to_string(nreads)).c_str());
  }
  else if (strcmp(message, "IP") == 0)
  {
    report_ip();
  }
  else
  {
    mqtt_client.publish(mqtt_out_e, "Didn't understand message");
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
      report_ip();
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

void report_ip()
{
  char buf[18];
  String s = WiFi.localIP().toString();
  s.toCharArray(buf, 18);
  mqtt_client.publish(mqtt_out_d, buf);
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
  // fast value to see if we're changin
  float grams = scale.get_units(nreads);
  mqtt_client.publish(mqtt_out_kg, std::to_string(grams / 1000.f).c_str());
}

void scale_calibrate(float cal_weight_grams, float tolerance)
{
  // start to zero grams
  scale.tare();

  // get the original scale value
  float orig_scale = scale.get_scale();

  float units;
  float start_units = scale.get_units(3);

  uint32_t timerStart = millis();
  while (abs((units = scale.get_units(3)) - start_units) < 0.5 * cal_weight_grams)
  {
    mqtt_client.publish(mqtt_out_d, ("Place calibration weight - " + std::to_string(cal_weight_grams) + " grams").c_str());
    if ((millis() - timerStart) > 10 * 1000)
    {
      mqtt_client.publish(mqtt_out_e, "Calibration timed out");
      return;
    }
    delay(500);
  }

  uint8_t cal_success = 0;
  float calib;
  float step = 2;
  float value = scale.get_value(5);
  float guess = cal_weight_grams / abs(value);

  bool increasing_guess = true;
  bool decreasing_guess = false;
  uint8_t sign_flip_count = 0;

  for (int i = 1; i <= 50; ++i)
  {
    scale.set_scale(guess);
    units = scale.get_units(i > 4 ? 15 : 8);
    mqtt_client.publish(mqtt_out_d, ("Optimising | units = " + std::to_string(units) + " | guess = " + std::to_string(guess)).c_str());

    if (abs(units - cal_weight_grams) < tolerance)
    {
      mqtt_client.publish(mqtt_out_d, "Success");
      cal_success++;
      calib = guess;
      step /= 5.0f;
      if (cal_success > 3)
        break;
    }
    else
    {
      if (sign_flip_count == 3)
      {
        step = 0.9 * step;
        sign_flip_count = 0;
        mqtt_client.publish(mqtt_out_d, ("sign flip detected | decreasing step to " + std::to_string(step)).c_str());
      }
      if (cal_weight_grams > units)
      {
        guess = guess < step ? guess / 5.f : guess - step;
        mqtt_client.publish(mqtt_out_d, "undershoot");
        if (increasing_guess)
        {
          ++sign_flip_count;
          increasing_guess = false;
          decreasing_guess = true;
        }
      }
      else
      {
        guess += step;
        mqtt_client.publish(mqtt_out_d, "overshoot");
        if (decreasing_guess)
        {
          ++sign_flip_count;
          decreasing_guess = false;
          increasing_guess = true;
        }
      }
    }
  }

  // store to EEPROM
  if (cal_success)
  {
    EEPROM.put(scale_calibration_address, calib);
    if (EEPROM.commit())
      mqtt_client.publish(mqtt_out_d, "Stored calibration to EEPROM");
    else
    {
      mqtt_client.publish(mqtt_out_e, "Saving to EEPROM failed!");
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
