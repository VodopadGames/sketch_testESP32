//Libraries for BLE
#include <BLEDevice.h>
#include <BLEServer.h>

// Library for working with DS3231 RTC(Real Time Clock)
//DS3231 works with I2C protocol
//ESP32 standard I2C pins:
//
//    SDA (data) → GPIO21
//
//    SCL (clock) → GPIO22
//
//    VCC (power) → 3.3V or 5V (ESP32 suports both)
//
//    GND (ground) → GND


#include <Wire.h>
#include <RTClib.h>

// Library for working with DHT humidity/temperature sensor
// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
#include <DHT.h>

//Definitions for working with soil moisture sensor
#define SOIL_SENSOR_PIN 33 // ESP32 pin GPIO34 (ADC6) that connects to AOUT pin of moisture sensor
// Limits (Has to be calibrated!)
#define DRY_VALUE 3000  // Dry soil
#define WET_VALUE 1500  // Wet soil
#define MIN_SOIL_MOISTURE 20 // Below this water
#define MAX_SOIL_MOISTURE 70 // Above this stop watering

//Definitions for control pins (temporary diodes)
#define HEATER_RED_LED 18     // GPIO for red diode
#define COOLER_BLUE_LED 19    // GPIO for blue diode
#define VENTILATION_GREEN_LED 23  //GPIO for green diode
#define PUMP_YELLOW_LED 13 //GPIO for yellow diode
#define LIGHT_WHITE_LED 14 //GPIO for white diode

//Definitions for temperature/humidity control
#define MIN_TEMPERATURE 24 // Below this switch on heating
#define MAX_TEMPERATURE 26 // Above this switch on cooling
#define MAX_HUMIDITY 70 // Above this start ventilation

//Definitions for light control
#define ON_HOUR 14       // Hour for light on
#define ON_MINUTE 11     // Minute for light on
#define OFF_HOUR 14       // Hour for light off
#define OFF_MINUTE 12    // Minute for light off

// Digital pin connected to the DHT sensor
#define AIR_TEMP_HUMIDITY_PIN 4     
// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

//BLE definitions
#define BUTTON_PIN 32  // Button pin for the ON BLE button. Must be touched to GND to work.
#define LED_PIN 2 // Internal LED on GPIO 2

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(AIR_TEMP_HUMIDITY_PIN, DHTTYPE);

RTC_DS3231 rtc;

//Variables for BLE use
bool bleActive = false;
bool deviceConnected = false;
BLEServer* pServer = NULL;
unsigned long lastActiveTime = 0;  // Global variable to track last BLE activity time

// Callback class to track BLE connection status
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("# # # BLE: BLE Device Connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("# # # BLE: BLE Device Disconnected");
        pServer->startAdvertising(); // Restart advertising
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println(F("Start!"));
  
  dht.begin();

  //set the control pins
  pinMode(HEATER_RED_LED, OUTPUT);
  pinMode(COOLER_BLUE_LED, OUTPUT);
  pinMode(VENTILATION_GREEN_LED, OUTPUT);
  pinMode(PUMP_YELLOW_LED, OUTPUT);
  pinMode(LIGHT_WHITE_LED, OUTPUT);

  //set the indicator LED pin and button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED OFF

  digitalWrite(HEATER_RED_LED, LOW);
  digitalWrite(COOLER_BLUE_LED, LOW);
  digitalWrite(VENTILATION_GREEN_LED, LOW);
  digitalWrite(PUMP_YELLOW_LED, LOW);
  digitalWrite(LIGHT_WHITE_LED, LOW);

  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);

  Wire.begin(21, 22); // Инициализиране на I2C със SDA = GPIO21, SCL = GPIO22

  if (!rtc.begin()) {
      Serial.println("# # # RTC: Не е намерен RTC модул!");
      while (1);
  }

  if (rtc.lostPower()) {
      Serial.println("# # # RTC: Захранването на RTC е загубено, настройване на време!");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Автоматично задаване на текущото време от компютъра
  }
}

void loop() {

  // Switch on / off BLE
   if (digitalRead(BUTTON_PIN) == LOW && !bleActive) {
        Serial.println();
        Serial.println("# # # BLE: Button pressed! Starting BLE...");
        Serial.println();
        startBLE();
    }

    if (bleActive) {
        if (deviceConnected) {
            digitalWrite(LED_PIN, HIGH); // LED ON when connected
        } else {
            // Blink LED when waiting for a connection
            digitalWrite(LED_PIN, LOW);
            delay(500);
            digitalWrite(LED_PIN, HIGH);
            delay(500);
        }

        // Stop BLE after 30 seconds of no connection
        if (!deviceConnected && millis() - lastActiveTime > 30000) {
            Serial.println();
            Serial.println("# # # BLE: No connection, stopping BLE...");
            Serial.println();
            stopBLE();
        }
    }

  // Wait a few seconds between measurements.
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();
  // Read the analog value from soil moisture sensor
  int soilSensorValue = analogRead(SOIL_SENSOR_PIN);

  // Conversion to soil moisture percent from sensor readings
  float soilMoisture = 100.0 * (DRY_VALUE - soilSensorValue) / (DRY_VALUE - WET_VALUE);
    
  // Limiting to 0 - 100%
  soilMoisture = constrain(soilMoisture, 0, 100);


  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("# # # DHT sensor: Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(temperature, humidity, false);

  Serial.println();
  Serial.print(F("# # # DHT sensor: Temperature: "));
  Serial.print(temperature);
  Serial.println(F("°C"));

  Serial.print(F("# # # DHT sensor: Humidity: "));
  Serial.print(humidity);
  Serial.println(F("%"));

  Serial.print(F("# # # DHT sensor: Heat index: "));
  Serial.print(hic);
  Serial.println(F("°C "));

  Serial.print(F("# # # Soil sensor: Soil moisture value: "));
  Serial.print(soilMoisture);
  Serial.print(F("%  "));

  Serial.print(F("raw value: "));
  Serial.println(soilSensorValue);

  DateTime now = rtc.now();

  int currentHour = now.hour();
  int currentMinute = now.minute();
  
  Serial.print("# # # RTC: Дата: ");
  Serial.print(now.day());
  Serial.print(".");
  Serial.print(now.month());
  Serial.print(".");
  Serial.print(now.year());
  Serial.print(" ");

  Serial.print("Час: ");
  Serial.print(currentHour);
  Serial.print(":");
  Serial.print(currentMinute);
  Serial.print(":");
  Serial.print(now.second());
  Serial.println();

  bool isTimeToTurnOn = (currentHour > ON_HOUR) || (currentHour == ON_HOUR && currentMinute >= ON_MINUTE);
  bool isTimeToTurnOff = (currentHour > OFF_HOUR) || (currentHour == OFF_HOUR && currentMinute >= OFF_MINUTE);

  if (isTimeToTurnOn && !isTimeToTurnOff) {
      digitalWrite(LIGHT_WHITE_LED, HIGH);  // Включва диода
      Serial.println("# # # RTC: Диодът е включен.");
  } else {
      digitalWrite(LIGHT_WHITE_LED, LOW);   // Изключва диода
      Serial.println("# # # RTC: Диодът е изключен.");
  }
  
  if (temperature < MIN_TEMPERATURE) {
      digitalWrite(HEATER_RED_LED, HIGH);
      digitalWrite(COOLER_BLUE_LED, LOW);
  } 
  else if (temperature > MAX_TEMPERATURE) {
      digitalWrite(HEATER_RED_LED, LOW);
      digitalWrite(COOLER_BLUE_LED, HIGH);
  } 
  else {
      digitalWrite(HEATER_RED_LED, LOW);
      digitalWrite(COOLER_BLUE_LED, LOW);
  }

  if(humidity > MAX_HUMIDITY) {
      digitalWrite(VENTILATION_GREEN_LED, HIGH);
  }
  else {
      digitalWrite(VENTILATION_GREEN_LED, LOW);
  }

  if(soilMoisture < MIN_SOIL_MOISTURE) {
      digitalWrite(PUMP_YELLOW_LED, HIGH);
  }
  else if(soilMoisture > MAX_SOIL_MOISTURE) {
      digitalWrite(PUMP_YELLOW_LED, LOW);
  }
}

void startBLE() {
    if (bleActive) return; // Prevent restarting if already active

    BLEDevice::init("# # # BLE: ESP32_BLE_Device");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pService = pServer->createService(BLEUUID((uint16_t)0x180F));

    BLECharacteristic* pCharacteristic = pService->createCharacteristic(
        BLEUUID((uint16_t)0x2A19), 
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->setValue("# # # BLE: Hello from ESP32!");
    pService->start();
    
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->start();

    // Reset the BLE timeout timer
    lastActiveTime = millis();

    bleActive = true;
    Serial.println();
    Serial.println("# # # BLE: BLE started.");
    Serial.println();
}

void stopBLE() {
    if (pServer) {
        pServer->getAdvertising()->stop();
        pServer = nullptr; // Reset pointer
    }
    
    bleActive = false;
    deviceConnected = false;
    digitalWrite(LED_PIN, LOW); // Turn LED OFF when BLE stops
    Serial.println("# # # BLE: BLE stopped.");
}

