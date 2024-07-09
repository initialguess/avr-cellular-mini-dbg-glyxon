/**
 * Environmental Sensor Network for Desert Botanical Gardens and 
 * Glyxon Biolabs Microfludics Project
 * 
 * This Arduino sketch is the main program for an environmental sensor network.
 * It collects data from various sensors and publishes it via MQTT over LTE.
 * 
 * Hardware:
 * - Arduino-compatible board with LTE capabilities
 * - SCD40 CO2 sensors (x2: test and control)
 * - SHT31 temperature and humidity sensor
 * - Atlas Scientific pH sensor
 * - VH400 soil moisture sensor
 * - Rain gauge
 * - VEML3328 color sensor (onboard but disabled in this setup)
 * - MCP9808 temperature sensor (onboard but disabled in this setup)
 * 
 * The sketch performs the following main functions:
 * 1. Initializes all sensors and communication interfaces
 * 2. Reads data from active sensors
 * 3. Publishes sensor data via MQTT
 * 4. Enters low power mode between readings
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SensirionI2CScd4x.h>
#include <DFRobot_SHT3x.h>
#include <mcp9808.h>
#include <veml3328.h>
#include <ecc608.h>
#include <led_ctrl.h>
#include <log.h>
#include <low_power.h>
#include <lte.h>
#include <mqtt_client.h>
#include <stdint.h>
#include "application.h"
#include "credentials.h"

/**
 * Setup function
 * 
 * This function is called once when the Arduino boots up.
 * It initializes all the sensors, communication interfaces,
 * and prepares the device for operation.
 */
void setup() {
    // Initialize LED controller and run startup animation
    LedCtrl.begin();
    LedCtrl.startupCycle();

    // Initialize serial communication
    SerialModule.begin(115200);
    SerialModule.setTimeout(120000);
    delay(2000);  // Allow time for serial to initialize

    // Initialize I2C communication
    Wire1.begin();

    // Setup rain gauge interrupt
    rainTipping_setup();

    // Initialize temperature and humidity sensors
    sht3x_init();
    scd4x_init(&scd4x_control, SCD_CONTROL_MUX, SCD_ALTITUDE);
    scd4x_init(&scd4x_test, SCD_TEST_MUX, SCD_ALTITUDE);

    // Configure low power mode
    LowPower.configurePowerDown();

    // Initialize and then disable unused onboard sensors to save power
    Veml3328.begin();
    Mcp9808.begin();
    Veml3328.shutdown();
    Mcp9808.shutdown();

    // Turn off pH sensor LED and put it to sleep
    ezo_led_off_then_sleep();

    SerialModule.println("Setup complete");

    // Initialize LTE connection
    while (!Lte.begin()) {
        // Keep trying to establish LTE connection
    }
}

/**
 * Main loop function
 * 
 * This function runs repeatedly after setup() completes.
 * It performs the main tasks of reading sensors, publishing data,
 * and entering low power mode.
 */
void loop() {
    // Read data from all active sensors
    readSensors();

    // Publish collected sensor data via MQTT
    publishMQTTData();

    // Enter low power mode to conserve energy between readings
    enterLowPower();
}
