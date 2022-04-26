#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Credentials.h>
#include <ESP8266WiFi.h>
#include <HX711.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <string>

// follows the guide on https://honey-pi.de/en/4x-half-bridge-waegezellen/

// status LED
static const int led_pin = 2; // GPIO2

// the scale's ADC
HX711 scale;
static const int scale_dout_pin = 3; // GPIO2
static const int scale_sck_pin = 1;  // GPIO0
static const long scale_offset = 50682624;
static const long scale_divider = 5895655;

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

void setup()
{
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);

  ticker.attach_ms(100, blink_red_led);

  setup_serial();
  setup_wifi();
  setup_ota();
  setup_hx711();
  setup_mqtt();

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

  if (scale.wait_ready_timeout(1000))
  {
    long reading = scale.get_units(10);
    mqtt_client.publish("scale/out/w", std::to_string(reading).c_str());
  }
  else
  {
    Serial.println("HX711 not found.");
    mqtt_client.publish("scale/out/e", "HX711 not found");
  }

  delay(2000);
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
  scale.begin(scale_dout_pin, scale_sck_pin);

  scale.set_scale(scale_divider);
  scale.set_offset(scale_offset);
}

void setup_mqtt()
{
  mqtt_client.setServer(mqtt_server, MOSQUITTO_PORT);
  mqtt_client.setCallback(mqtt_callback);
  mqtt_connect();
}

void mqtt_callback(char *topic, byte *payload, uint16_t length)
{
  Serial.println("MQTT callback");
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
      mqtt_client.publish("scale/out/status", "scale ready");
      // ... and resubscribe
      mqtt_client.subscribe(mqtt_in_topic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.println(mqtt_client.state());
      // Wait 5 seconds before retrying
      delay(1000);
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
