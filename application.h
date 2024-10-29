/**
 * Environmental Sensor Network for Desert Botanical Gardens and 
 * Glyxon Biolabs Microfludics Project
 * 
 * This project integrates multiple sensors to collect environmental data,
 * including CO2 levels, temperature, humidity, pH, soil moisture, and rainfall.
 * It uses MQTT to publish the collected data.
 * 
 * Hardware:
 * - AVR Cellular Mini
 * - SCD40 CO2 sensors (x2: test and control)
 * - SHT31 temperature and humidity sensor
 * - Atlas Scientific pH sensor
 * - VH400 soil moisture sensor
 * - Rain gauge
 * - PCA9548A I2C multiplexer (to resolve I2C address conflict)
 * - I2C Extender
 * 
 * Libraries used:
 * - ArduinoJson 
 * - SensirionI2CScd4x
 * - DFRobot_SHT3x
 * - Various custom libraries (mcp9808, veml3328, ecc608, led_ctrl, log, low_power, lte, mqtt_client)
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

#include "credentials.h"

extern "C" {
  #include "adc.h"
}

// Configuration and pin definitions
#define SerialModule Serial3
// #define DEBUG  // Uncomment this line to enable debug prints

// Pin definitions
#define VH400_PD3 ADC_MUXPOS_AIN3_gc
#define BATTERY ADC_MUXPOS_AIN8_gc
#define VOLTAGE_MEASURE_EN_PIN PIN_PB3
#define VOLTAGE_MEASURE_PIN PIN_PE0
#define RAIN_PIN PIN_PE1

// I2C addresses and multiplexer channels
#define PCAADDR 0x70
#define PH_EZO_ADDRESS 0x63
#define SHT_ADDRESS 0x44
#define EZO_PH_MUX 0
#define SCD_CONTROL_MUX 3
#define SCD_TEST_MUX 4
#define SHT_CONTROLMUX 7
#define SCD_ALTITUDE 391  // Altitude in meters (adjust as needed)

// MQTT configuration
#define MQTT_SUB_TOPIC "mqtt-test"
#define MQTT_PUB_TOPIC "mqtt-test"
#define MQTT_THING_NAME "node1"
#define MQTT_PORT 8883
#define MQTT_USE_TLS true
#define MQTT_USE_ECC false
#define MQTT_KEEPALIVE false
#define PUB_INTERVAL_SEC 3600  // Publish interval in seconds

// pH variables and array
byte code = 0;
char ph_data[32];
byte in_char = 0;
byte i = 0;
float ph_float;

// Rain Tipping Software Debounce
#define DEBOUNCE_INTERVAL 10 

// Sensor data structure
typedef struct {
    float co2_test;
    float airTemp_test;
    float humidity_test;
    float co2_ctrl;
    float airTemp_ctrl;
    float humidity_ctrl;
    float pH;
    float moisture;
    float soilTemp;
    float battery;
    float rain;
} sensors;

// Global variables
sensors sensorData;
DFRobot_SHT3x sht3x(&Wire1, SHT_ADDRESS, 4);
SensirionI2CScd4x scd4x_control;
SensirionI2CScd4x scd4x_test;
volatile uint8_t tips = 0;
unsigned long lastDebounceTime = 0;

// Function prototypes
void rainTipping_setup(void);
void calculateRainfall(void);
void pcaselect(uint8_t i);
void sht3x_init(void);
void sht3x_getMeasurements(void);
void readSHT31_C(void);
void scd4x_init(SensirionI2CScd4x *scd4x, uint8_t mux, uint16_t sensorAltitude);
void readSCD40(SensirionI2CScd4x *scd4x, uint8_t mux);
void readBattery(void);
void readVH400(void);
void ezo_ph_read_cal_temperature(void);
void ezo_led_off_then_sleep(void);
void readSensors(void);
size_t retrieveData(char* data, const size_t data_capacity);
void publishMQTTData(void);
void enterLowPower(void);

/**
 * Rainfall Measurement Functions
 */

void rainfall_callback(void) {
    if (millis() - lastDebounceTime > DEBOUNCE_INTERVAL) {
        tips++;
        lastDebounceTime = millis();
    }
}

void rainTipping_setup(void) {
    pinConfigure(RAIN_PIN, PIN_DIR_INPUT | PIN_PULLUP_ON);
    attachInterrupt(RAIN_PIN, rainfall_callback, FALLING);
}

void calculateRainfall(void) {
    sensorData.rain = tips * 0.2794;  // Convert tips to millimeters
    tips = 0;
}

/**
 * I2C Multiplexer Functions
 */

void pcaselect(uint8_t i) {
    if (i > 7) return;
    Wire1.beginTransmission(PCAADDR);
    Wire1.write(1 << i);
    Wire1.endTransmission();
}

/**
 * SHT31 Temperature and Humidity Sensor Functions
 */

void sht3x_init(void) {
    pcaselect(SHT_CONTROLMUX);
    while(sht3x.begin() != 0) {
        #ifdef DEBUG
            SerialModule.println("Failed to initialize SHT31, check wiring");
        #endif
        delay(1000);
    }
}

void readSHT31_C(void) {
    pcaselect(SHT_CONTROLMUX);
    sensorData.soilTemp = sht3x.getTemperatureC();
}

/**
 * SCD40 CO2 Sensor Functions
 */

void scd4x_init(SensirionI2CScd4x *scd4x, uint8_t mux, uint16_t sensorAltitude) {
    pcaselect(mux);
    uint16_t error;

    scd4x->begin(Wire1);

    error = scd4x->stopPeriodicMeasurement();
    if (error) {
        #ifdef DEBUG
            SerialModule.println("Error stopping SCD40 measurement");
        #endif
    }

    error = scd4x->setSensorAltitude(sensorAltitude);
    if (error) {
        #ifdef DEBUG
            SerialModule.println("Error setting SCD40 altitude");
        #endif
    }

    error = scd4x->startPeriodicMeasurement();
    if (error) {
        #ifdef DEBUG
            SerialModule.println("Error starting SCD40 measurement");
        #endif
    }
}

void readSCD40(SensirionI2CScd4x *scd4x, uint8_t mux) {
    uint16_t error;
    uint16_t co2 = 0;
    float temperature = 0.2f;
    float humidity = 0.2f;
    bool isDataReady = false;

    pcaselect(mux);

    error = scd4x->startPeriodicMeasurement();
    #ifdef DEBUG
      if (error) {
          SerialModule.println(F("Error trying to execute startPeriodicMeasurement()"));
      }
    #endif

    while (!isDataReady) {
        error = scd4x->getDataReadyFlag(isDataReady);
        if (error) {
          #ifdef DEBUG
            SerialModule.println(F("Error trying to execute getDataReadyFlag():"));
          #endif  
        } else if (isDataReady) {
          // Data is ready, proceed with processing
          break;
        } else {
          // Data not ready, wait and try again
          delay(1000);
        }
    }

    error = scd4x->readMeasurement(co2, temperature, humidity);
    
    if (error) {
      #ifdef DEBUG
        SerialModule.println(F("Error trying to execute readMeasurement()"));
      #endif  
    } else if (co2 == 0) {
      #ifdef DEBUG
        SerialModule.println(F("Invalid sample detected, skipping."));
      #endif
    } 
    
    else {
        if(mux == SCD_CONTROL_MUX) {
          sensorData.co2_ctrl = co2;
          sensorData.airTemp_ctrl = temperature;
          sensorData.humidity_ctrl = humidity;
        }
        if(mux == SCD_TEST_MUX) {
          sensorData.co2_test = co2;
          sensorData.airTemp_test = temperature;
          sensorData.humidity_test = humidity;
        }
    }
    error = scd4x->stopPeriodicMeasurement();
    if (error) {
      #ifdef DEBUG
        SerialModule.println(F("Error trying to execute stopPeriodicMeasurement()"));
      #endif
    }
}

/**
 * Battery Measurement Function
 */

void readBattery(void) {
  uint16_t adc_read;

  ADC_Disable();

  if (!digitalRead(VOLTAGE_MEASURE_EN_PIN)) {
        pinConfigure(VOLTAGE_MEASURE_EN_PIN, PIN_DIR_OUTPUT);
        digitalWrite(VOLTAGE_MEASURE_EN_PIN, HIGH);
  }

  ADC_Init(VREF_REFSEL_VDD_gc, 0, 0, ADC_CONVMODE_SINGLEENDED_gc, 0, ADC_RESSEL_12BIT_gc, 
        0, ADC_INITDLY_DLY16_gc, ADC_SAMPNUM_NONE_gc, ADC_PRESC_DIV2_gc, ADC_SAMPDLY_DLY12_gc,
        2, BATTERY, ADC_MUXNEG_GND_gc, ADC_WINCM_NONE_gc, 0, 0);

  ADC_Enable();
  delay(1000);

  ADC_Start_Conversion();
  while(!(ADC_Is_Conversion_Done()));
  adc_read = ADC_Get_Result();
  sensorData.battery = adc_read * 3.3f * 4.0f / 4096.0f;
  #ifdef DEBUG
    SerialModule.print(F("Battery: "));
    SerialModule.println(sensorData.battery);
  #endif

  // For power conservation, disable PE0, PB3, and the ADC after measurement
  PORTE.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
  ADC_Disable();

}

/**
 * Soil Moisture Measurement Functions
 */

float calculateVWC(float voltage) {
  float VWC;
  if(voltage <= 1.1) {
    VWC = (10 * voltage);
  }
  else if(voltage > 1.1 && voltage <= 1.3) {
    VWC = (25 * voltage) - 17.5;
  }
  else if(voltage > 1.3 && voltage <= 1.82) {
    VWC = (48.08 * voltage) - 47.5;
  }
  else if(voltage > 1.82) {
    VWC = (26.32 * voltage) - 7.89;
  }

  return VWC;
}

void readVH400(void) {
  uint16_t adc_read;
  float voltage;

  ADC_Disable();

  ADC_Init(VREF_REFSEL_2V500_gc, 0, 0, ADC_CONVMODE_SINGLEENDED_gc, 0, ADC_RESSEL_12BIT_gc, 
          0, ADC_INITDLY_DLY16_gc, ADC_SAMPNUM_NONE_gc, ADC_PRESC_DIV2_gc, ADC_SAMPDLY_DLY0_gc,
          2, VH400_PD3, ADC_MUXNEG_GND_gc, ADC_WINCM_NONE_gc, 0, 0);

  ADC_Enable();

  delay(1000);

  ADC_Start_Conversion();
  while(!(ADC_Is_Conversion_Done()));
  adc_read = ADC_Get_Result();
  voltage = adc_read * (2.500 / 4096);
  sensorData.moisture = calculateVWC(voltage); 
}

/**
 * pH Measurement Functions
 */
void ezo_print_ph() {
  Wire1.requestFrom(PH_EZO_ADDRESS, 32, 1); //call the circuit and request 32 bytes (this may be more than we need)
  code = Wire1.read();               		    //the first byte is the response code, we read this separately.

  #ifdef DEBUG
    switch (code) {
      case 1:							                    //switch case based on what the response code is.
        //SerialModule.println("Success");    		      //means the command was successful.
        break;                        		    //exits the switch case.

      case 2:                         		    //decimal 2.
        SerialModule.println("Failed");     		          //means the command has failed.
        break;                        	 	    //exits the switch case.

      case 254:                       		    //decimal 254.
        SerialModule.println("Pending");    		      //means the command has not yet been finished calculating.
        break;                        		    //exits the switch case.

      case 255:                       		    //decimal 255.
        SerialModule.println("No Data");    		          //means there is no further data to send.
        break;                         		    //exits the switch case.
    }
  #endif

  if(code < 255) {
    while (Wire1.available()) {         		//are there bytes to receive.
      in_char = Wire1.read();           		//receive a byte.
      ph_data[i] = in_char;					        //load this byte into our array.
      i += 1;                          		  //incur the counter for the array element.
      if (in_char == 0) {              		  //if we see that we have been sent a null command.
        i = 0;                         		  //reset the counter i to 0.
        break;                         		  //exit the while loop.
      }
    }

    //Convert it into floating point number.
    ph_float=atof(ph_data);
    #ifdef DEBUG
      SerialModule.print("pH: ");
      SerialModule.println(ph_data);
    #endif
  } else {};
}

void ezo_led_off_then_sleep(void) {
  pcaselect(EZO_PH_MUX);
  Wire1.beginTransmission(PH_EZO_ADDRESS);  
  Wire1.write("L,0");  
  Wire1.endTransmission();

  delay(300);
  
  Wire1.beginTransmission(PH_EZO_ADDRESS);  
  Wire1.write("Sleep");  
  Wire1.endTransmission();  
}

void ezo_ph_read_cal_temperature(void) {
  char ph_buffer[20];
  int stringLength;

  stringLength = sprintf(ph_buffer, "rt,%.1f", (double)sensorData.soilTemp);
  pcaselect(EZO_PH_MUX);
  Wire1.beginTransmission(PH_EZO_ADDRESS);  
  
  for (int i = 0; i < stringLength; i++) {
    Wire1.write(ph_buffer[i]);
  }

  Wire1.endTransmission();
    delay(900); 
  Wire1.requestFrom(PH_EZO_ADDRESS, 32, 1); //call the circuit and request 32 bytes (this may be more than we need)
  code = Wire1.read();               		    //the first byte is the response code, we read this separately.
  #ifdef DEBUG
    switch (code) {
      case 1:							                    //switch case based on what the response code is.
        //SerialModule.println("Success");    		      //means the command was successful.
        break;                        		    //exits the switch case.

      case 2:                         		    //decimal 2.
        SerialModule.println("Failed");     		          //means the command has failed.
        break;                        	 	    //exits the switch case.

      case 254:                       		    //decimal 254.
        SerialModule.println("Pending");    		      //means the command has not yet been finished calculating.
        break;                        		    //exits the switch case.

      case 255:                       		    //decimal 255.
        SerialModule.println("No Data");    		          //means there is no further data to send.
        break;                         		    //exits the switch case.
    }
  #endif

  if(code < 255) {
    while (Wire1.available()) {         		//are there bytes to receive.
      in_char = Wire1.read();           		//receive a byte.
      ph_data[i] = in_char;					        //load this byte into our array.
      i += 1;                          		  //incur the counter for the array element.
      if (in_char == 0) {              		  //if we see that we have been sent a null command.
        i = 0;                         		  //reset the counter i to 0.
        break;                         		  //exit the while loop.
      }
    }

    //Convert it into floating point number.
    ph_float=atof(ph_data);

    sensorData.pH = ph_float;

  } else {}; 
}

void ezo_sleep() {
  pcaselect(EZO_PH_MUX);
  Wire1.beginTransmission(PH_EZO_ADDRESS);  
  Wire1.write("Sleep");  
  Wire1.endTransmission();
}

void ezo_led_off() {
  pcaselect(EZO_PH_MUX);
  Wire1.beginTransmission(PH_EZO_ADDRESS);  
  Wire1.write("L,0");  
  Wire1.endTransmission();

  delay(300);
}


/**
 * Sensor Reading and Data Publishing Functions
 */

void readSensors() {
    readSHT31_C();
    readSCD40(&scd4x_control, SCD_CONTROL_MUX);
    readSCD40(&scd4x_test, SCD_TEST_MUX);
    ezo_ph_read_cal_temperature();
    readVH400();
    readBattery();
    calculateRainfall();

    #ifdef DEBUG
        printSensorData();
    #endif

    pcaselect(1);  // Switch to unused I2C MUX to prevent waking the core
}

size_t retrieveData(char* data, const size_t data_capacity) {
  StaticJsonDocument<96> payload;
  JsonArray jsonArray = payload.to<JsonArray>();

  /* Prepare JSON Data Payload */
  JsonObject jsonItem = jsonArray.createNestedObject();
  jsonItem["c_t"] = sensorData.co2_test;
  jsonItem["a_t"] = sensorData.airTemp_test;
  jsonItem["h_t"] = sensorData.humidity_test;
  jsonItem["c_c"] = sensorData.co2_ctrl;
  jsonItem["a_c"] = sensorData.airTemp_ctrl;
  jsonItem["h_c"] = sensorData.humidity_ctrl;
  jsonItem["p"] = sensorData.pH;
  jsonItem["v"] = sensorData.moisture;
  jsonItem["s"] = sensorData.soilTemp;
  jsonItem["b"] = sensorData.battery;
  jsonItem["r"] = sensorData.rain;

  return serializeJson(jsonArray, data, data_capacity);
}

void publishMQTTData(void) {
  while (!Lte.begin()) {}

  if (Lte.isConnected()) {
    /* Check if connected to the broker.  If not, attempt to connect */
    if (!MqttClient.isConnected()) {
        if (MqttClient.begin(MQTT_THING_NAME,
                        MQTT_BROKER,
                        MQTT_PORT,
                        MQTT_USE_TLS,
                        MQTT_KEEPALIVE,
                        MQTT_USE_ECC,
                        MQTT_USER,
                        MQTT_PASS)) {
          MqttClient.subscribe(MQTT_SUB_TOPIC);
        } else {
          #ifdef DEBUG
            SerialModule.println(F("\r\n"));
            SerialModule.println(F("Failed to connect to broker"));
          #endif  
        } 
    }

    static char data[1024] = "";

    if (retrieveData(data, sizeof(data)) > sizeof(data)) {
      #ifdef DEBUG
        SerialModule.println(F("Data buffer too small."));
      #endif
      while (1) {}
    }

    Log.infof(F("Publishing data: %s\r\n"), data);
    if (!MqttClient.publish(MQTT_PUB_TOPIC, data)) {
      #ifdef DEBUG
        SerialModule.println(F("Failed to publish data"));
       #endif 
    }
    #ifdef DEBUG
      SerialModule.print(F("payload size: "));
      SerialModule.println(sizeof(data));
    #endif
    }
}

/**
 * Power Management Function
 */

void enterLowPower(void) {
  // Switch to an unused I2C MUX to prevent it from waking up the core
  pcaselect(1);
  
  #ifdef DEBUG
    SerialModule.println("Powering down...");
    // Delay some to allow the log message to be transmitted before the device is going into power save mode
    delay(100);
  #endif

  uint32_t remaining = PUB_INTERVAL_SEC;

  /* To keep a regular publishing cadence and avoid publishing early if interrupted during rainfall,
     the CPU is returned to sleep for any remaining time */
  while(remaining != 0) {
    remaining = LowPower.powerDown(remaining);
    #ifdef DEBUG 
      if (remaining > 0) {
        SerialModule.println(remaining);
        delay(10);
      }
    #endif     
  }
  SerialModule.println("Leaving low power");
}