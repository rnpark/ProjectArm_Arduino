#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>

#if not defined (_VARIANT_ARDUINO_DUE_X_)
  #include <SoftwareSerial.h>
#endif

#include <Wire.h>
extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <utility/quaternion.h>
#include <EEPROM.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"



// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* Set the delay between fresh samples (not too fast, BLE UART is slow!) */
/* Firware <=0.6.6 should use 500ms, >=0.6.7 can use 200ms */
#define BNO055_SAMPLERATE_DELAY_MS (100)

//#include "Wire.h"


#define TCAADDR 0x70


Adafruit_BNO055 bno1 = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55);
Adafruit_BNO055 bno3 = Adafruit_BNO055(55);

void tcaselect(uint8_t i) {
	if (i > 7) return;

	Wire.beginTransmission(TCAADDR);
	Wire.write(1 << i);
	Wire.endTransmission();
}


/**************************************************************************/
/*!
    @brief  A small helper function for error messages
*/
/**************************************************************************/
void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tcaselect(0);
  bno1.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");

  tcaselect(1);
  bno2.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");

  tcaselect(2);
  bno3.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");

  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  // Get the system status values (mostly for debugging purposes)
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
 
  // Display the results in the Serial Monitor
  tcaselect(0);
  bno1.getSystemStatus(&system_status, &self_test_results, &system_error);
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");

  // Display the 2nd BNO results in the Serial Monitor
  tcaselect(1);
  bno2.getSystemStatus(&system_status, &self_test_results, &system_error);
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");

  // Display the 3rd BNO results in the Serial Monitor
  tcaselect(2);
  bno3.getSystemStatus(&system_status, &self_test_results, &system_error);
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  // Get the four calibration values (0..3)
  // 3 means 'fully calibrated"
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  tcaselect(1);
  bno2.getCalibration(&system, &gyro, &accel, &mag);

  // The data should be ignored until the system calibration is > 0
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  // Display the 1st sensor value
  Serial.print("ReferenceABS");
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);

  tcaselect(0);
  bno1.getCalibration(&system, &gyro, &accel, &mag);
  // Display the 2nd sensor value
  Serial.print("ShoulderABS");
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);

  tcaselect(2);
  bno3.getCalibration(&system, &gyro, &accel, &mag);
  // Display the 3rd sensor value
  Serial.print("ElbowABS");
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
//Check Calibration on 1st BNO
bool isDevCal_1(void)
{
  tcaselect(0);
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno1.getCalibration(&system, &gyro, &accel, &mag);
  if(system == 3 && gyro == 3 && accel >= 2 && mag >= 2)
  {
    return true;
  }
  return false;
}

//Check Calibration on 2nd BNO
bool isDevCal_2(void)
{
	tcaselect(1);
	uint8_t system, gyro, accel, mag;
	system = gyro = accel = mag = 0;
	bno2.getCalibration(&system, &gyro, &accel, &mag);
	if (system == 3 && gyro == 3 && accel >= 2 && mag >= 2)
	{
		return true;
	}
	return false;
}
//Check Calibration on 3rd BNO
bool isDevCal_3(void)
{
	tcaselect(2);
	uint8_t system, gyro, accel, mag;
	system = gyro = accel = mag = 0;
	bno3.getCalibration(&system, &gyro, &accel, &mag);
	if (system == 3 && gyro == 3 && accel >= 2 && mag >= 2)
	{
		return true;
	}
	return false;
}


/**************************************************************************/
/*
    Sends sensor calibration status out over BLE UART
*/
/**************************************************************************/
void transmitCalStatus(void)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  // Get the four calibration values (0..3)
  // 3 means 'fully calibrated"
  bno1.getCalibration(&system, &gyro, &accel, &mag);
  /* Prepare the AT command */
  ble.print("AT+BLEUARTTX=");

  // Transmit individual values
  // Note: The values are abbreviated compared to the Serial Monitor
  // to save space since BLE UART is quite slow */
  ble.print(",");
  ble.print(system, DEC);
  ble.print(gyro, DEC);
  ble.print(accel, DEC);
  ble.println(mag, DEC);

  bno2.getCalibration(&system, &gyro, &accel, &mag);
  ble.print("AT+BLEUARTTX=");

  // Transmit individual values
  // Note: The values are abbreviated compared to the Serial Monitor
  // to save space since BLE UART is quite slow */
  ble.print(",");
  ble.print(system, DEC);
  ble.print(gyro, DEC);
  ble.print(accel, DEC);
  ble.println(mag, DEC);

  //Get calibration data from 3rd BNO
  bno3.getCalibration(&system, &gyro, &accel, &mag);
  ble.print("AT+BLEUARTTX=");

  // Transmit individual values
  // Note: The values are abbreviated compared to the Serial Monitor
  // to save space since BLE UART is quite slow */
  ble.print(",");
  ble.print(system, DEC);
  ble.print(gyro, DEC);
  ble.print(accel, DEC);
  ble.println(mag, DEC);
  if (! ble.waitForOK() )
  {
    Serial.println(F("Failed to send?"));
  }
}

/**************************************************************************/
/*!
    @brief  Initializes the one wire temperature sensor
*/ 
/**************************************************************************/
void initSensor(void)
{
	tcaselect(0);
  if(!bno1.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, 1st BNO055 not detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //bno1.setExtCrystalUse(true);

  tcaselect(1);
  if (!bno2.begin())
  {
	  // There was a problem detecting the BNO055 ... check your connections
	  Serial.print("Ooops, 2nd BNO055 not detected ... Check your wiring or I2C ADDR!");
	  while (1);
  }
  //bno2.setExtCrystalUse(true);

  tcaselect(2);
  if (!bno3.begin())
  {
	  // There was a problem detecting the BNO055 ... check your connections
	  Serial.print("Ooops, 3rd BNO055 not detected ... Check your wiring or I2C ADDR!");
	  while (1);
  }
 // bno3.setExtCrystalUse(true);
  delay(1000);

  // Display some basic information on this sensor
  //displaySensorDetails();

  // Optional: Display current status
  //displaySensorStatus();

  

}

/**************************************************************************/
/*
Display the raw calibration offset and radius data
*/
/**************************************************************************/

void viewMultiplexChannels() {
	//*************Begin MULTIPLEXER text*********//
	Serial.println("\nTCAScanner ready!");

	for (uint8_t t = 0; t<8; t++) {
		tcaselect(t);
		Serial.print("TCA Port #"); Serial.println(t);

		for (uint8_t addr = 0; addr <= 127; addr++) {
			if (addr == TCAADDR) continue;

			uint8_t data;
			if (!twi_writeTo(addr, &data, 0, 1, 1)) {
				Serial.print("Found I2C 0x");  Serial.println(addr, HEX);
			}
		}
	}
	Serial.println("\ndone");
	//*********End of MULTIPLEXER text***********//
}
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

//****Shoulder Sensor****//
void loadOffsets_1()
{
	tcaselect(0);
	int eeAddress = 0;
	long bnoID;
	bool foundCalib = false;

	EEPROM.get(eeAddress, bnoID);

	adafruit_bno055_offsets_t calibrationData;
	sensor_t sensor;

	//
	//  Look for the sensor's unique ID at the beginning oF EEPROM.
	//  This isn't foolproof, but it's better than nothing.
	//

	
	bno1.getSensor(&sensor);
	if (bnoID != sensor.sensor_id)
	{
		Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
		delay(500);
	}
	else
	{
		Serial.println("\nFound Calibration for this sensor in EEPROM.");
		eeAddress += sizeof(long);
		EEPROM.get(eeAddress, calibrationData);

		//displaySensorOffsets(calibrationData);

		Serial.println("\n\nRestoring Calibration data to the BNO055...");
		bno1.setSensorOffsets(calibrationData);

		Serial.println("\n\nCalibration data loaded into BNO055");
		foundCalib = true;
	}

	delay(1000);

	// Display some basic information on this sensor 
	//displaySensorDetails();

	// Optional: Display current status 
	//displaySensorStatus();

	bno1.setExtCrystalUse(true);

	sensors_event_t event;
	bno1.getEvent(&event);
	if (foundCalib) {
		Serial.println("Move sensor slightly to calibrate magnetometers");
		while (!bno1.isFullyCalibrated())
		{
			bno1.getEvent(&event);
			delay(BNO055_SAMPLERATE_DELAY_MS);
		}
	}
	else
	{
		Serial.println("Please Calibrate Sensor: ");
		while (!bno1.isFullyCalibrated())
		{
			bno1.getEvent(&event);

			Serial.print("X: ");
			Serial.print(event.orientation.x, 4);
			Serial.print("\tY: ");
			Serial.print(event.orientation.y, 4);
			Serial.print("\tZ: ");
			Serial.print(event.orientation.z, 4);

			// Optional: Display calibration status 
			displayCalStatus();

			// New line for the next sample 
			Serial.println("");

			// Wait the specified delay before requesting new data 
			delay(BNO055_SAMPLERATE_DELAY_MS);
		}
	}

	Serial.println("\nFully calibrated!");
	Serial.println("--------------------------------");
	Serial.println("Calibration Results: ");
	adafruit_bno055_offsets_t newCalib;
	bno1.getSensorOffsets(newCalib);
	//displaySensorOffsets(newCalib);

	Serial.println("\n\nStoring calibration data to EEPROM...");

	eeAddress = 0;
	bno1.getSensor(&sensor);
	bnoID = sensor.sensor_id;

	EEPROM.put(eeAddress, bnoID);

	eeAddress += sizeof(long);
	EEPROM.put(eeAddress, newCalib);
	Serial.println("Data stored to EEPROM.");

	Serial.println("\n--------------------------------\n");
	delay(500);
}
//*****Reference Sensor*****//
void loadOffsets_2()
{
	tcaselect(1);
	int eeAddress = 1;
	long bnoID;
	bool foundCalib = false;

	EEPROM.get(eeAddress, bnoID);

	adafruit_bno055_offsets_t calibrationData;
	sensor_t sensor;

	//
	//  Look for the sensor's unique ID at the beginning oF EEPROM.
	//  This isn't foolproof, but it's better than nothing.
	//
	
	bno2.getSensor(&sensor);
	if (bnoID != sensor.sensor_id)
	{
		Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
		delay(500);
	}
	else
	{
		Serial.println("\nFound Calibration for this sensor in EEPROM.");
		eeAddress += sizeof(long);
		EEPROM.get(eeAddress, calibrationData);

		//displaySensorOffsets(calibrationData);

		Serial.println("\n\nRestoring Calibration data to the BNO055...");
		bno2.setSensorOffsets(calibrationData);

		Serial.println("\n\nCalibration data loaded into BNO055");
		foundCalib = true;
	}

	delay(1000);

	// Display some basic information on this sensor 
	//displaySensorDetails();

	// Optional: Display current status 
	//displaySensorStatus();

	bno2.setExtCrystalUse(true);

	sensors_event_t event;
	bno2.getEvent(&event);
	if (foundCalib) {
		Serial.println("Move sensor slightly to calibrate magnetometers");
		while (!bno2.isFullyCalibrated())
		{
			bno2.getEvent(&event);
			delay(BNO055_SAMPLERATE_DELAY_MS);
		}
	}
	else
	{
		Serial.println("Please Calibrate Sensor: ");
		while (!bno2.isFullyCalibrated())
		{
			bno2.getEvent(&event);

			Serial.print("X: ");
			Serial.print(event.orientation.x, 4);
			Serial.print("\tY: ");
			Serial.print(event.orientation.y, 4);
			Serial.print("\tZ: ");
			Serial.print(event.orientation.z, 4);

			// Optional: Display calibration status 
			displayCalStatus();

			// New line for the next sample 
			Serial.println("");

			// Wait the specified delay before requesting new data 
			delay(BNO055_SAMPLERATE_DELAY_MS);
		}
	}

	Serial.println("\nFully calibrated!");
	Serial.println("--------------------------------");
	Serial.println("Calibration Results: ");
	adafruit_bno055_offsets_t newCalib;
	bno2.getSensorOffsets(newCalib);
	//displaySensorOffsets(newCalib);

	Serial.println("\n\nStoring calibration data to EEPROM...");

	eeAddress = 1;
	bno2.getSensor(&sensor);
	bnoID = sensor.sensor_id;

	EEPROM.put(eeAddress, bnoID);

	eeAddress += sizeof(long);
	EEPROM.put(eeAddress, newCalib);
	Serial.println("Data stored to EEPROM.");

	Serial.println("\n--------------------------------\n");
	delay(500);
}
//******Elbow Sensor*****//
void loadOffsets_3()
{
	tcaselect(2);
	int eeAddress = 2;
	long bnoID;
	bool foundCalib = false;

	EEPROM.get(eeAddress, bnoID);

	adafruit_bno055_offsets_t calibrationData;
	sensor_t sensor;

	//
	//  Look for the sensor's unique ID at the beginning oF EEPROM.
	//  This isn't foolproof, but it's better than nothing.
	//
	
	bno3.getSensor(&sensor);
	if (bnoID != sensor.sensor_id)
	{
		Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
		delay(500);
	}
	else
	{
		Serial.println("\nFound Calibration for this sensor in EEPROM.");
		eeAddress += sizeof(long);
		EEPROM.get(eeAddress, calibrationData);

		//displaySensorOffsets(calibrationData);

		Serial.println("\n\nRestoring Calibration data to the BNO055...");
		bno3.setSensorOffsets(calibrationData);

		Serial.println("\n\nCalibration data loaded into BNO055");
		foundCalib = true;
	}

	delay(1000);

	// Display some basic information on this sensor 
	//displaySensorDetails();

	// Optional: Display current status 
	//displaySensorStatus();

	bno3.setExtCrystalUse(true);

	sensors_event_t event;
	bno3.getEvent(&event);
	if (foundCalib) {
		Serial.println("Move sensor slightly to calibrate magnetometers");
		while (!bno3.isFullyCalibrated())
		{
			bno3.getEvent(&event);
			delay(BNO055_SAMPLERATE_DELAY_MS);
		}
	}
	else
	{
		Serial.println("Please Calibrate Sensor: ");
		while (!bno3.isFullyCalibrated())
		{
			bno3.getEvent(&event);

			Serial.print("X: ");
			Serial.print(event.orientation.x, 4);
			Serial.print("\tY: ");
			Serial.print(event.orientation.y, 4);
			Serial.print("\tZ: ");
			Serial.print(event.orientation.z, 4);

			// Optional: Display calibration status 
			displayCalStatus();

			// New line for the next sample 
			Serial.println("");

			// Wait the specified delay before requesting new data 
			delay(BNO055_SAMPLERATE_DELAY_MS);
		}
	}

	Serial.println("\nFully calibrated!");
	Serial.println("--------------------------------");
	Serial.println("Calibration Results: ");
	adafruit_bno055_offsets_t newCalib;
	bno3.getSensorOffsets(newCalib);
	//displaySensorOffsets(newCalib);

	Serial.println("\n\nStoring calibration data to EEPROM...");

	eeAddress = 2;
	bno3.getSensor(&sensor);
	bnoID = sensor.sensor_id;

	EEPROM.put(eeAddress, bnoID);

	eeAddress += sizeof(long);
	EEPROM.put(eeAddress, newCalib);
	Serial.println("Data stored to EEPROM.");

	Serial.println("\n--------------------------------\n");
	delay(500);
}

/*
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
	Serial.print("Accelerometer: ");
	Serial.print(calibData.accel_offset_x); Serial.print(" ");
	Serial.print(calibData.accel_offset_y); Serial.print(" ");
	Serial.print(calibData.accel_offset_z); Serial.print(" ");

	Serial.print("\nGyro: ");
	Serial.print(calibData.gyro_offset_x); Serial.print(" ");
	Serial.print(calibData.gyro_offset_y); Serial.print(" ");
	Serial.print(calibData.gyro_offset_z); Serial.print(" ");

	Serial.print("\nMag: ");
	Serial.print(calibData.mag_offset_x); Serial.print(" ");
	Serial.print(calibData.mag_offset_y); Serial.print(" ");
	Serial.print(calibData.mag_offset_z); Serial.print(" ");

	Serial.print("\nAccel Radius: ");
	Serial.print(calibData.accel_radius);

	Serial.print("\nMag Radius: ");
	Serial.print(calibData.mag_radius);
}
*/

void setup(void)
{
	//for Multiplexer to determine addresses connected
	Wire.begin();

	//Baud rate for serial output
	Serial.begin(9600);
	delay(1000);

	Serial.println("Orientation Sensor Test"); Serial.println("");

	if (!ble.begin(VERBOSE_MODE))
	{
		error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
	}

	// Disable command echo from Bluefruit
	ble.echo(false);

	Serial.println("Requesting Bluefruit info:");
	// Print Bluefruit information
	ble.info();

	ble.verbose(false);  // debug info is a little annoying after this point!

	/* Initialise the sensor */

	initSensor();
	loadOffsets_1();  //Shoulder Sensor
	loadOffsets_2();  //Reference Sensor
	loadOffsets_3();  //Elbow Sensor
	//if (!bno.begin())
	//{
		// There was a problem detecting the BNO055 ... check your connections 
		//Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		//while (1);
	//}

	
	//Make sure the 3 IMU's are calibrated before transmitting orientation data.
	do {
		// set_calibration();
		isDevCal_1();
		isDevCal_2();
		isDevCal_3();
		displayCalStatus();
		ble.print("Continue Cal");
		Serial.print("Continue Cal");
	} while (!isDevCal_1() && !isDevCal_2 && !isDevCal_3);
	transmitCalStatus();
	//get_calibration();
	Serial.println("Calibration Complete");
	ble.print("Cal Complete");

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
	if(ble.isConnected())
	{
    /*
		//Reference Sensor formatted to a max of 20 chars
		tcaselect(1);
		imu::Vector<3> euler2 = bno2.getVector(Adafruit_BNO055::VECTOR_EULER);
		// Display the floating point data 

		ble.print("AT+BLEUARTTXF="); //Allows us to flush the buffer after loading the x,y,z values from the reference sensor.
		ble.print("RF");
		ble.print(euler2.x(), 1);
		ble.print(euler2.y(), 1);
		ble.println(euler2.z(), 1);  //Buffer is flushed after the println.

		
		Serial.print("REF Sensor:");
		Serial.print("X: ");
		Serial.print(euler2.x(), 1);
		Serial.print(", ");
		Serial.print("Y: ");
		//ble.print("AT+BLEUARTTXF=");
		//ble.print(",");
		Serial.print(euler2.y(), 1);
		Serial.print(", ");
		Serial.print("Z: ");
		Serial.println(euler2.z(), 1);
		
		//Shoulder Sensor formatted to a max of 20 chars
		tcaselect(0);
		imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
		// Display the floating point data 
		ble.print("AT+BLEUARTTXF=");
		ble.print("MA");
		ble.print(euler1.x(), 1);
		ble.print(euler1.y(), 1);
		ble.println(euler1.z(), 1);

		
		// Display the floating point data to serial 
		Serial.print("SHO Sensor:");
		Serial.print("X: ");
		Serial.print(euler1.x(), 1);
		Serial.print(", ");
		Serial.print("Y: ");
		//ble.print("AT+BLEUARTTXF=");
		//ble.print(",");
		Serial.print(euler1.y(), 1);
		Serial.print(", ");
		Serial.print("Z: ");
		Serial.println(euler1.z(), 1);
		

		//Elbow Sensor formatted to a max of 20 chars
		tcaselect(2);
		imu::Vector<3> euler3 = bno3.getVector(Adafruit_BNO055::VECTOR_EULER);
		// Display the floating point data 
		ble.print("AT+BLEUARTTXF=");
		ble.print("EX");
		ble.print(euler3.x(), 1);
		ble.print(euler3.y(), 1);
		//ble.print(",");
		ble.println(euler3.z(), 1);
		
		
		Serial.print("ELB Sensor:");
		Serial.print("X: ");
		Serial.print(euler3.x(), 1);
		Serial.print(", ");
		Serial.print("Y: ");
		//ble.print("AT+BLEUARTTXF=");
		//ble.print(",");
		Serial.print(euler3.y(), 1);
		Serial.print(", ");
		Serial.print("Z: ");
		Serial.println(euler3.z(), 1);
    */


	// Get Quaternion data (no 'Gimbal Lock' like with Euler angles)


		tcaselect(1);
		imu::Quaternion quat2 = bno2.getQuat();
		// Send abbreviated integer data out over BLE UART
    quat2.normalize();
    float temp2 = quat2.x();
    quat2.x() = -quat2.y();
    quat2.y() = temp2;
    quat2.z() = -quat2.z();
    imu::Vector<3> euler2 = quat2.toEuler();

    ble.print("AT+BLEUARTTXF=");
    ble.print("RF");
    ble.print(-180/M_PI * euler2.x(), 1);
    ble.print(180/M_PI * euler2.y(), 1);
    ble.println(180/M_PI * euler2.z(), 1);

    Serial.print("Reference sensor: ");
    Serial.print("X: ");
    Serial.print(-180/M_PI * euler2.x(), 1);
    Serial.print(", Y: ");
    Serial.print(180/M_PI * euler2.y(), 1);
    Serial.print(", Z: ");
    Serial.println(180/M_PI * euler2.z(), 1);
    /*
		ble.print("AT+BLEUARTTXF=");
		ble.print("Rw");
		ble.print(quat.w(), 4);
		ble.print("Rx");
		ble.println(quat.x(), 4);
		//ble.print(",");
		ble.print("AT+BLEUARTTXF=");
		ble.print("Ry");
		ble.print(quat.y(), 4);
		ble.print("Rz");
		ble.println(quat.z(), 4);

		Serial.print("Refernce Sensor:");
		Serial.print("W: ");
		Serial.print(quat.w(), 4);
		Serial.print(", ");
		Serial.print("X: ");
		//ble.print("AT+BLEUARTTXF=");
		//ble.print(",");
		Serial.print(quat.x(), 4);
		Serial.print(", ");
		Serial.print("Y: ");
		Serial.print(quat.y(), 4);
		Serial.print(", ");
		Serial.print("Z: ");
		Serial.println(quat.z(), 4);
    */
    
    tcaselect(0);
	  imu::Quaternion quat1 = bno1.getQuat();
	  // Send abbreviated integer data out over BLE UART
    quat1.normalize();
    float temp1 = quat1.x();
    quat1.x() = -quat1.y();
    quat1.y() = temp1;
    quat1.z() = -quat1.z();
    imu::Vector<3> euler1 = quat1.toEuler();

    ble.print("AT+BLEUARTTXF=");
    ble.print("MA");
    ble.print(-180/M_PI * euler1.x(), 1);
    ble.print(180/M_PI * euler1.y(), 1);
    ble.println(180/M_PI * euler1.z(), 1);
    
    Serial.print("Shoulder sensor: ");
    Serial.print("X: ");
    Serial.print(-180/M_PI * euler1.x(), 1);
    Serial.print(", Y: ");
    Serial.print(180/M_PI * euler1.y(), 1);
    Serial.print(", Z: ");
    Serial.println(180/M_PI * euler1.z(), 1);
    /*
  	ble.print("AT+BLEUARTTXF=");
  	ble.print("Sw");
  	ble.print(quat.w(), 4);
  	ble.print("Sx");
  	ble.println(quat.x(), 4);
  	//ble.print(",");
  	ble.print("AT+BLEUARTTXF=");
  	ble.print("Sy");
  	ble.print(quat.y(), 4);
  	ble.print("Sz");
  	ble.println(quat.z(), 4);
  
  	Serial.print("Shoulder Sensor:");
  	Serial.print("W: ");
  	Serial.print(quat.w(), 4);
  	Serial.print(", ");
  	Serial.print("X: ");
  	//ble.print("AT+BLEUARTTXF=");
  	//ble.print(",");
  	Serial.print(quat.x(), 4);
  	Serial.print(", ");
  	Serial.print("Y: ");
  	Serial.print(quat.y(), 4);
  	Serial.print(", ");
  	Serial.print("Z: ");
  	Serial.println(quat.z(), 4);
    */  
  
  	tcaselect(2);
  	imu::Quaternion quat3 = bno3.getQuat();
  	// Send abbreviated integer data out over BLE UART
    quat3.normalize();
    float temp3 = quat3.x();
    quat3.x() = -quat3.y();
    quat3.y() = temp3;
    quat3.z() = -quat3.z();
    imu::Vector<3> euler3 = quat3.toEuler();
    
    ble.print("AT+BLEUARTTXF=");
    ble.print("EX");
    ble.print(-180/M_PI * euler3.x(), 1);
    ble.print(180/M_PI * euler3.y(), 1);
    ble.println(180/M_PI * euler3.z(), 1);
    
    Serial.print("Elbow sensor: ");
    Serial.print("X: ");
    Serial.print(-180/M_PI * euler3.x(), 1);
    Serial.print(", Y: ");
    Serial.print(180/M_PI * euler3.y(), 1);
    Serial.print(", Z: ");
    Serial.println(180/M_PI * euler3.z(), 1);

    /*
  	ble.print("AT+BLEUARTTXF=");
  	ble.print("Ew");
  	ble.print(quat.w(), 4);
  	ble.print("Ex");
  	ble.println(quat.x(), 4);
  	//ble.print(",");
  	ble.print("AT+BLEUARTTXF=");
  	ble.print("Ey");
  	ble.print(quat.y(), 4);
  	ble.print("Ez");
  	ble.println(quat.z(), 4);
  
  	Serial.print("Elbow Sensor:");
  	Serial.print("W: ");
  	Serial.print(quat.w(), 4);
  	Serial.print(", ");
  	Serial.print("X: ");
  	//ble.print("AT+BLEUARTTXF=");
  	//ble.print(",");
  	Serial.print(quat.x(), 4);
  	Serial.print(", ");
  	Serial.print("Y: ");
  	Serial.print(quat.y(), 4);
  	Serial.print(", ");
  	Serial.print("Z: ");
  	Serial.println(quat.z(), 4);
    */
  	//displayCalStatus();
  	
	/*
	// Send a new line character for the next record
		//ble.println("AT+BLEUARTTX=\\r\\n");
	//if (!ble.waitForOK())
	//{
		//Serial.println(F("Failed to send?"));
	//}
  */
	//displayCalStatus();
	//Serial.println("");
	delay(100);
}
	
	//displayCalStatus();
	//Serial.println("");

  
}

