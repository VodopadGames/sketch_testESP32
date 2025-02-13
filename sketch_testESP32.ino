// Code for working with DS3231 RTC(Real Time Clock)
#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-soil-moisture-sensor
 */

#define SOIL_SENSOR_PIN 34 // ESP32 pin GPIO34 (ADC6) that connects to AOUT pin of moisture sensor

// Граници (калибрирай за твоя сензор)
#define DRY_VALUE 3000  // Стойност при суха почва
#define WET_VALUE 1500  // Стойност при влажна почва
#define MIN_SOIL_MOISTURE 20 // Под това поливай
#define MAX_SOIL_MOISTURE 70 // Над това спри поливането

// Example testing sketch for various DHT humidity/temperature sensors written by ladyada
// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <DHT.h>

#define HEATER_RED_LED 18     // GPIO за червен диод
#define COOLER_BLUE_LED 19    // GPIO за син диод
#define VENTILATION_GREEN_LED 23  //GPIO за зелен диод
#define PUMP_YELLOW_LED 15 //GPIO за жълт диод
#define LIGHT_WHITE_LED 2  //GPIO за бял диод

#define MIN_TEMPERATURE 24 // Below this swith on heating
#define MAX_TEMPERATURE 26 // Above this swith on cooling
#define MAX_HUMIDITY 70 // Above this start ventilation

#define ON_HOUR 14       // Час за включване на диода
#define ON_MINUTE 11     // Минута за включване на диода
#define OFF_HOUR 14       // Час за изключване на диода
#define OFF_MINUTE 12    // Минута за изключване на диода


#define AIR_TEMP_HUMIDITY_PIN 4     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

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

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(AIR_TEMP_HUMIDITY_PIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  Serial.println(F("DHT22 test!"));
  Serial.println(F("HW-390 test!"));

  dht.begin();

  pinMode(HEATER_RED_LED, OUTPUT);
  pinMode(COOLER_BLUE_LED, OUTPUT);
  pinMode(VENTILATION_GREEN_LED, OUTPUT);
  pinMode(PUMP_YELLOW_LED, OUTPUT);
  pinMode(LIGHT_WHITE_LED, OUTPUT);

  digitalWrite(HEATER_RED_LED, LOW);
  digitalWrite(COOLER_BLUE_LED, LOW);
  digitalWrite(VENTILATION_GREEN_LED, LOW);
  digitalWrite(PUMP_YELLOW_LED, LOW);
  digitalWrite(LIGHT_WHITE_LED, LOW);

  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);

  Wire.begin(21, 22); // Инициализиране на I2C със SDA = GPIO21, SCL = GPIO22

  if (!rtc.begin()) {
      Serial.println("Не е намерен RTC модул!");
      while (1);
  }

  if (rtc.lostPower()) {
      Serial.println("Захранването на RTC е загубено, настройване на време!");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Автоматично задаване на текущото време от компютъра
  }
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();
  // Read the analog value from soil moisture sensor
  int soilSensorValue = analogRead(SOIL_SENSOR_PIN);

  // Преобразуване в процент влажност
  float soilMoisture = 100.0 * (DRY_VALUE - soilSensorValue) / (DRY_VALUE - WET_VALUE);
    
  // Ограничаване в границите 0 - 100%
  soilMoisture = constrain(soilMoisture, 0, 100);


  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(temperature, humidity, false);

  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print(F("%  Temperature: "));
  Serial.print(temperature);
  Serial.print(F("°C  Moisture value: "));
  Serial.println(soilMoisture);
  Serial.print(F("%  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(soilSensorValue);

  DateTime now = rtc.now();

  int currentHour = now.hour();
  int currentMinute = now.minute();
  
  Serial.print("Дата: ");
  Serial.print(now.day());
  Serial.print(".");
  Serial.print(now.month());
  Serial.print(".");
  Serial.print(now.year());
  Serial.print(" ");
  Serial.print(currentHour);
  Serial.print(":");
  Serial.print(currentMinute);
  Serial.print(":");
  Serial.println(now.second());

  bool isTimeToTurnOn = (currentHour > ON_HOUR) || (currentHour == ON_HOUR && currentMinute >= ON_MINUTE);
  bool isTimeToTurnOff = (currentHour > OFF_HOUR) || (currentHour == OFF_HOUR && currentMinute >= OFF_MINUTE);

  if (isTimeToTurnOn && !isTimeToTurnOff) {
      digitalWrite(LIGHT_WHITE_LED, HIGH);  // Включва диода
      Serial.println("Диодът е включен.");
  } else {
      digitalWrite(LIGHT_WHITE_LED, LOW);   // Изключва диода
      Serial.println("Диодът е изключен.");
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
