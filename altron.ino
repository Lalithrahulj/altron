/*
Company: LR Robotic Labs
Developer: Lalith_RJ
Date: 10-12-2024
*/

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include "Adafruit_GFX.h"
#include "OakOLED.h"
#include "DHT.h"

// OLED HEADER
OakOLED oled;

// Pulse Oximeter
#define REPORTING_PERIOD_MS 1000
PulseOximeter pox;

// DHT Sensor
#define DHTPIN 5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// FreeRTOS handles
TaskHandle_t readTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t dhtTaskHandle;
TaskHandle_t hrMonitorTaskHandle; // Heart rate monitor task handle

// Shared variables
volatile float heartRate = 0;
volatile float spO2 = 0;
volatile float temperature = 0;
volatile float humidity = 0;
SemaphoreHandle_t dataMutex;

// Thresholds for heart rate
float heartRateThresholdLow = 50;
float heartRateThresholdHigh = 100;
bool ledState = false; // To keep track of LED state

// Callback for pulse oximeter beat detection
void onBeatDetected()
{
    Serial.println("Beat!");
}

// Task for reading data from the pulse oximeter
void readTask(void *pvParameters)
{
    uint32_t tsLastReport = 0;
    pinMode(12, OUTPUT);  // Pin 12 for blinking
    pinMode(13, OUTPUT);  // Pin 13 for being kept high

    while (true)
    {
        pox.update();

        // Update shared variables under mutex protection
        if (millis() - tsLastReport > REPORTING_PERIOD_MS)
        {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            heartRate = pox.getHeartRate();
            spO2 = pox.getSpO2();
            xSemaphoreGive(dataMutex);

            tsLastReport = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task for reading data from the DHT sensor
void dhtTask(void *pvParameters)
{
    while (true)
    {
        // Read temperature and humidity
        float h = dht.readHumidity();
        float t = dht.readTemperature();

        // Check for sensor failure
        if (!isnan(h) && !isnan(t))
        {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            humidity = h;
            temperature = t;
            xSemaphoreGive(dataMutex);
        }
        else
        {
            Serial.println("Failed to read from DHT sensor!");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Update every 2 seconds
    }
}

// Task for displaying data on the OLED
void displayTask(void *pvParameters)
{
    while (true)
    {
        // Get shared data under mutex protection
        float currentHeartRate, currentSpO2, currentTemp, currentHumidity;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        currentHeartRate = heartRate;
        currentSpO2 = spO2;
        currentTemp = temperature;
        currentHumidity = humidity;
        xSemaphoreGive(dataMutex);

        // Update the OLED display
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(1);

        // Pulse Oximeter data
        oled.setCursor(0, 0); // First row
        oled.print("HR: ");
        oled.print(currentHeartRate);
        oled.println(" bpm");

        oled.setCursor(0, 16); // Second row
        oled.print("SpO2: ");
        oled.print(currentSpO2);
        oled.println(" %");

        // DHT data on the same row
        oled.setCursor(0, 32); // Third row
        oled.print("T: ");
        oled.print(currentTemp);
        oled.print(" C ");
        oled.print("H: ");
        oled.print(currentHumidity);
        oled.println(" %");

        oled.display();

        vTaskDelay(pdMS_TO_TICKS(500)); // Update display every 500 ms
    }
}

// Task for monitoring heart rate and toggling the LED
void hrMonitorTask(void *pvParameters)
{
    while (true)
    {
        // Get the current heart rate under mutex protection
        float currentHeartRate;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        currentHeartRate = heartRate;
        xSemaphoreGive(dataMutex);

        // Check if the heart rate goes below 50
        if (currentHeartRate < heartRateThresholdLow && !ledState)
        {
            digitalWrite(12, HIGH);  // Turn LED on
            ledState = true;
        }
        // Check if the heart rate goes above 100
        else if (currentHeartRate > heartRateThresholdHigh && ledState)
        {
            digitalWrite(12, HIGH);  // Turn LED off
            ledState = true;
        } else {
          digitalWrite(12, LOW);
          ledState = false;
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Check every 500 ms
    }
}

void setup()
{
    Serial.begin(115200);

    // Initialize OLED
    oled.begin();
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(0, 0);
    oled.display();

    // Initialize pulse oximeter
    Serial.print("Initializing pulse oximeter...");
    if (!pox.begin())
    {
        Serial.println("FAILED");
        oled.clearDisplay();
        oled.println("FAILED");
        oled.display();
        for (;;);
    }
    else
    {
        Serial.println("SUCCESS");
        oled.clearDisplay();
        oled.println("SUCCESS");
        oled.display();
    }
    pox.setOnBeatDetectedCallback(onBeatDetected);

    // Initialize DHT sensor
    Serial.println("Initializing DHT sensor...");
    dht.begin();

    // Create a mutex for shared data
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL)
    {
        Serial.println("Failed to create mutex!");
        for (;;);
    }

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(readTask, "Read Task", 2048, NULL, 1, &readTaskHandle, 0);
    xTaskCreatePinnedToCore(dhtTask, "DHT Task", 2048, NULL, 1, &dhtTaskHandle, 0);
    xTaskCreatePinnedToCore(displayTask, "Display Task", 2048, NULL, 1, &displayTaskHandle, 1);
    xTaskCreatePinnedToCore(hrMonitorTask, "Heart Rate Monitor Task", 2048, NULL, 1, &hrMonitorTaskHandle, 0);
}

void loop()
{
    // FreeRTOS tasks are running; nothing needed in loop
}
