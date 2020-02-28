#include <SPI.h>                  // SPI
#include "SdFat.h"                // SD card
#include <SoftwareSerial.h>       // Software serial for I2C/SPI
#include <Wire.h>                 // For I2C
#include <Adafruit_Sensor.h>      // General Adafruit sensor library
#include <Adafruit_FXOS8700.h>    // Acceleromter and Magentometer
#include <Adafruit_FXAS21002C.h>  // Gyro
#include <Adafruit_BMP280.h>      // BMP280 - temperature, pressure, altitude

//--------------------------------------
// Define Constants

#define LOG_FILE_NAME ("phoenix_flight_new2.txt")

#define SAMPLERATE_DELAY_MS (1)
#define CHECK_LED_PIN (5)   // LED which turns on after all sensors are initialized
#define TEST_LED_PIN (6)   // LED for acceleration testing
#define ACCEL_THRESHOLD (20)   // m/s^2

#define USE_SD (true)

//--------------------------------------
// IMU Setup

  /* Assign a unique ID to the sensors */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

//--------------------------------------
// BMP280 Setup

// Create temp/pressure sensor with software SPI
#define BMP_SCK   14
#define BMP_MISO  8
#define BMP_MOSI  7
#define BMP_CS    9

//Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

//--------------------------------------
// SD Card setup

// Set USE_SDIO to zero for SPI card access. 
#define USE_SDIO 0

/*
 * Set DISABLE_CHIP_SELECT to disable a second SPI device.
 * For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
 * to 10 to disable the Ethernet controller.
 */
const int8_t DISABLE_CHIP_SELECT = -1;

SdFat sd;
File myFile;

const int chipSelect = 20;//10;


//--------------------------------------
// Variables
// Exponential average of x acceleration

// when this is >30, motor ignites
float accel_avg = 0;
float keep_rate = 0.95;
bool motor_lit = false;

//--------------------------------------

bool initializeSd();
void logDataToSD();
void logDataToConsole();
void writeLineToFile(String filename, String testLine);
void readFileToConsole(String filename);

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  // Only if working on computer
  // COMMENT OUT if powering on battery
  //while (!Serial) { delay(1); }
  Serial.println("-- Serial begin --");

  if (USE_SD) {
    // If SD fails to initialize, crash :(
    if (!initializeSd()) { 
      Serial.println("There was an error initializing the SD card :(");
      return; 
    }
  
    myFile = sd.open(LOG_FILE_NAME, FILE_WRITE);
    if (!myFile) {
      Serial.print("error opening ");
      Serial.println(LOG_FILE_NAME);
      digitalWrite(CHECK_LED_PIN, LOW);
    } else {
      Serial.print("Success opening ");
      Serial.println(LOG_FILE_NAME);
    }
  }

  //Try to initialize BMP
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }

  /* Initialise the sensor */
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }

  // initialize the digital pin as an output.
  pinMode(CHECK_LED_PIN, OUTPUT);
  pinMode(TEST_LED_PIN, OUTPUT); 
  
  digitalWrite(CHECK_LED_PIN, HIGH);   // turn the LED on

  readFileToConsole("test.txt");

}

void loop() {
//  logDataToConsole();
  digitalWrite(CHECK_LED_PIN, LOW);   // turn the LED low
  logDataToSD(); // This will turn the LED HIGH after a successful write so SD


  if (!motor_lit && accel_avg > ACCEL_THRESHOLD) {
    motor_lit = true;
  } else if (motor_lit && accel_avg < 3) {
    digitalWrite(TEST_LED_PIN, HIGH);
  }
  
  delay(SAMPLERATE_DELAY_MS);
}

/*
 * Initializes the SD Card at quarter speed
 * Returns true if initialization succeeded
 * 
 * Note: only one file may be opened at one time, so you
 * must close it before opening another file.
 */
bool initializeSd() {
  Serial.print("Initializing SD card...");

  //Initialize at quarter speed (may prevent various issues)
  if (!sd.begin(chipSelect, SPI_QUARTER_SPEED)) {
    Serial.println("initialization failed!");
    return false;
  }
  Serial.println("initialization done.");
  return true;
}

void logDataToConsole() {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  
  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  
  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
  Serial.println(" m");

  /* Get a new sensor event */
  sensors_event_t event;
   
  /* Display the results (acceleration is measured in m/s^2) */
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);                                                              
  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(aevent.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(aevent.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(aevent.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print(F("MAG   "));
  Serial.print("X: "); Serial.print(mevent.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(mevent.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mevent.magnetic.z); Serial.print("  ");Serial.println("uT");
//
//  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");
  
  Serial.println();
}

void logDataToSD() {
  // Here we create a single line to input into the log file
  String dataLog = "";

  dataLog += millis(); dataLog += ",";
  
  dataLog += bmp.readTemperature(); dataLog += ",";     // BMP temperature
  dataLog += bmp.readPressure(); dataLog += ",";        // BMP pressure
  dataLog += bmp.readAltitude(1013.25); dataLog += ","; // BMP altitude

  /* Get a new sensor event */
  sensors_event_t event;
  sensors_event_t aevent, mevent;
   
  /* Display the results (acceleration is measured in m/s^2) */
  accelmag.getEvent(&aevent, &mevent);
  dataLog += aevent.acceleration.x; dataLog += ","; // Accelerometer x
  dataLog += aevent.acceleration.y; dataLog += ","; // Accelerometer y
  dataLog += aevent.acceleration.z; dataLog += ","; // Accelerometer z

  accel_avg = keep_rate*accel_avg + (1-keep_rate)*aevent.acceleration.x;

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  dataLog += mevent.magnetic.x; dataLog += ",";  // Magnetometer x
  dataLog += mevent.magnetic.y; dataLog += ",";  // Magnetometer y
  dataLog += mevent.magnetic.z; dataLog += ",";  // Magnetometer z

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  dataLog += event.gyro.x; dataLog += ","; // Gryo x
  dataLog += event.gyro.y; dataLog += ","; // Gryo y
  dataLog += event.gyro.z;                 // Gryo z

  Serial.println(dataLog);
  
  // Write the log to the log file
  // The log file is in CSV format for easy data analysis
  if (USE_SD) {
    writeLineToFile(LOG_FILE_NAME, dataLog);
  }
}

/*
 * Writes a single given line to a file.
 * 
 * This method opens and closes the file on every write,
 * which may cause slowdown but protects data in case of
 * sudden power failure or some other issue.
 */
void writeLineToFile(String filename, String testLine) {
  
  // If file is successfully opened, proceed
  Serial.print("Writing to ");
  Serial.print(filename);
  Serial.print("...  ");

  myFile = sd.open(LOG_FILE_NAME, FILE_WRITE);
  if (!myFile) {
    Serial.print("error opening ");
    Serial.println(LOG_FILE_NAME);
    digitalWrite(CHECK_LED_PIN, LOW);
  } else {
    myFile.println(testLine);
      // Close the file:
//    myFile.close();
    Serial.println("done writing.");
    digitalWrite(CHECK_LED_PIN, HIGH);   // turn the LED on
  }
  

}

/*
 * This method reads the contents of a file to the Serial Monitor
 */
void readFileToConsole(String filename) {
  
  myFile = sd.open(filename);

  // If file is successfully opened, proceed
  if (myFile) {
    Serial.print(filename);
    Serial.println(":");
    
    // Read from the file  to console until done
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // Close the file:
    myFile.close();
  } else {
    // If the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(filename);
  }
}
