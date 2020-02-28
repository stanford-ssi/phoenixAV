#include <SPI.h>                  // SPI
#include "SdFat.h"                // SD card
#include <SoftwareSerial.h>       // Software serial for I2C/SPI
#include <Wire.h>                 // For I2C
#include <Adafruit_Sensor.h>      // General Adafruit sensor library
#include <Adafruit_LSM303_U.h>    // Acceleromter
#include <Adafruit_L3GD20_U.h>    // Acceleromter
#include <Adafruit_9DOF.h>        // Acceleromter
#include <Adafruit_Simple_AHRS.h> // Accelerometer
#include <Adafruit_BMP280.h>      // BMP280 - temperature, pressure, altitude
#include <Servo.h>                // Servo

//--------------------------------------
// Define Constants

#define LOG_FILE_NAME ("phoenix_flight.txt")

#define SAMPLERATE_DELAY_MS (1)
#define CHECK_LED_PIN (5)   // LED which turns on after all sensors are initialized
#define TEST_LED_PIN (23)   // LED for acceleration testing

#define SERVO_PIN (4)       // Pin for servo control
#define SERVO_START_POS (174)
#define SERVO_END_POS (142)

#define ACCEL_THRESHOLD_BOTTOM (-16)   // m/s^2
#define KEEP_RATE (0.98)     // for exponential average calculation
#define KEEP_RATE_LONG (0.998)     // for exponential average calculation
//#define ACCEL_TIME_MS (200)   // milliseconds of exceeding threshold
#define RESET_INTERVAL (60000)

#define USE_SD (false)

//--------------------------------------
// IMU Setup

  /* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

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
// Servo Setup

Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
int servo_pos = SERVO_START_POS;    // variable to store the servo position 

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

//--------------------------------------
// Variables
// Exponential average of x acceleration

// when this is >30, motor ignites
float accel_avg = 0;
float accel_avg_long = 0;
bool motor_lit = false;
bool has_staged = false;
int accel_start_time = -1;
long stage_time = -1;
int tick_count = 0;

const int chipSelect = 20;//10;

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
    Serial.println("Using sd card");
    // If SD fails to initialize, crash :(
    if (!initializeSd()) { 
      Serial.println("There was an error initializing the SD card :(");
      return; 
    }
  
    myFile = sd.open(LOG_FILE_NAME, FILE_WRITE);
    if (!myFile) {
      Serial.print("error opening ");
      Serial.println(LOG_FILE_NAME);
      digitalWrite(CHECK_LED_PIN, LOW);   // turn the LED on
    } else {
      Serial.println("Initialized file.");
    }
  }

  

  //Try to initialize BMP
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }

  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object 
  myservo.write(servo_pos);           // 132 degrees is the closed position, 90 is open

  // initialize the digital pin as an output.
  pinMode(CHECK_LED_PIN, OUTPUT);
  pinMode(TEST_LED_PIN, OUTPUT); 
  
  digitalWrite(CHECK_LED_PIN, HIGH);   // turn the LED on

  digitalWrite(TEST_LED_PIN, HIGH);
  delay(500);
  digitalWrite(TEST_LED_PIN, LOW);   // turn the LED on

  if (USE_SD) {
    readFileToConsole("test.txt");
  }

}

void loop() {
//  logDataToConsole();
  digitalWrite(CHECK_LED_PIN, LOW);   // turn the LED on
  logDataToSD();

  // move servo slowly once staged
  if (has_staged) {
    // 90 = closed. Starts at 132;
    if (servo_pos > SERVO_END_POS) {
      servo_pos -= 1;
      myservo.write(servo_pos);
    }

    if (millis() - stage_time > RESET_INTERVAL) {
      servo_pos = SERVO_START_POS;
      myservo.write(SERVO_START_POS);
      has_staged = false;
      motor_lit = false;
    }

    // Motor burnout
  } else if (motor_lit && accel_avg > -4) {

    digitalWrite(TEST_LED_PIN, HIGH);
    Serial.println("WOW!----------");
    has_staged = true;

    stage_time = millis();
    
  } else if (!motor_lit && accel_avg < ACCEL_THRESHOLD_BOTTOM) {
    motor_lit = true;
    digitalWrite(TEST_LED_PIN, LOW);
  } else {
    if (tick_count == 0) {
      digitalWrite(TEST_LED_PIN, HIGH);
    } else {
      digitalWrite(TEST_LED_PIN, LOW);
    }

    tick_count = (tick_count+1)%50;

    if (accel_avg_long > 9) {
      myservo.write(SERVO_END_POS);
      Serial.println("CLOSE");
    } else {
      myservo.write(SERVO_START_POS);
    }
    
  }

  Serial.println(accel_avg);
  
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
  accel.getEvent(&event);                                                                   
  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  Serial.print(F("MAG   "));
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
//
//  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");

  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));
  }
  
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
   
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);   
  dataLog += event.acceleration.x; dataLog += ","; // Accelerometer x
  dataLog += event.acceleration.y; dataLog += ","; // Accelerometer y
  dataLog += event.acceleration.z; dataLog += ","; // Accelerometer z

  accel_avg = (float)(KEEP_RATE)*accel_avg + (1.0-(float)(KEEP_RATE))*event.acceleration.x;
  accel_avg_long = (float)(KEEP_RATE_LONG)*accel_avg_long + (1.0-(float)(KEEP_RATE_LONG))*event.acceleration.x;

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  dataLog += event.magnetic.x; dataLog += ",";  // Magnetometer x
  dataLog += event.magnetic.y; dataLog += ",";  // Magnetometer y
  dataLog += event.magnetic.z; dataLog += ",";  // Magnetometer z

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  dataLog += event.gyro.x; dataLog += ","; // Gryo x
  dataLog += event.gyro.y; dataLog += ","; // Gryo y
  dataLog += event.gyro.z; dataLog += ","; // Gryo z

  // Use the simple AHRS function to get the current orientation.
  sensors_vec_t   orientation;
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    dataLog += orientation.roll; dataLog += ",";    // AHRS roll
    dataLog += orientation.pitch; dataLog += ",";   // AHRS pitch
    dataLog += orientation.heading;                 // AHRS yaw
  }

//  Serial.println(dataLog);
//  Serial.println("");
  
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
  
  Serial.println((uint32_t)(myFile.println(testLine)));
  
  // Done writing
  Serial.println("done writing.");
  digitalWrite(CHECK_LED_PIN, HIGH);   // turn the LED on
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
