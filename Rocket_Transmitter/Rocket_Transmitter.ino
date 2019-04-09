/**************************************************************************/
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include <TinyGPS++.h>

#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>

#include <RH_E32.h>
//#include <e32.h>

#include <FS.h>
#include <SD.h>
/**************************************************************************/
//Definitions

#define BAUD 9600

RH_E32  driver(&Serial, 12, 25, 36);
/**************************************************************************/
// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// The TinyGPS++ object
TinyGPSPlus gps;

/**************************************************************************/
#define OS_PACK __attribute__((packed)) //ensure the compiler leaves no spacing in bitstream to COSMOS

#pragma pack(1)
typedef struct {
  //uint32_t length;
  uint32_t ID;
  uint32_t packet;
  float latitude;
  float longitude;
  float roll;
  float pitch;
  float yaw;
  float accelx;
  float accely;
  float accelz;
  float magx;
  float magy;
  float magz;
  float gyrox;
  float gyroy;
  float gyroz;
  float temperature;
  float pressure;
  float altitude;
  uint32_t endID;
} telemetry_packet;


telemetry_packet data;


struct sensorValidate {
  bool fail;
  bool init;
  int numfail;
};


//Declare Variables

unsigned int packetCounter = 0;
int logcounter = 0;
bool progstart = true;

//time for SD logging
uint32_t mytime;


static char outstr[15];

long lastMsg = 0;
long last = 0;
bool sd = true;

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

//Keep track of sensor failures
struct sensorValidate lsmlog;
struct sensorValidate bmplog;


/**************************************************************************/
void setup()
{

  //Initialize Serial
  Serial.begin(BAUD, SERIAL_8N1, 3, 1);

  while (!Serial) {
    //Serial.println("Serial cannot be opened");
    delay(1);
  }

  Serial2.begin(BAUD, SERIAL_8N1, 23, 19);

  while (!Serial2) {
    //Serial.println("Serial2 cannot be opened");
    delay(1);
  }


  //setup e32 uart radio
  while (!driver.init(1)) {
    //Serial.println("e32 init failed");
    delay(1);
  }


  lsmlog.fail = false;
  lsmlog.init = false;
  lsmlog.numfail = 0;


  bmplog.fail = false;
  bmplog.init = false;
  bmplog.numfail = 0;

  //lsm9ds1 and bmp180 initialization
  initSensors();

  //SD logging
  pinMode(14, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);

  SPI.begin(14, 2, 15, 13);

  if (!SD.begin(13, SPI, 40000000, "/sd")) {
    sd = false;
    SPI.end();
  }

  //initialize packet IDs
  data.ID = 0xDEAD;
  data.endID = 0xBEEF;

}


/**************************************************************************/

void initSensors()
{
  /* Initialise the sensor */
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    //Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    bmplog.fail = true;
    bmplog.init = true;
    bmplog.numfail++;
    logerror(1);
    while (1);
  }

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    //Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    lsmlog.fail = true;
    lsmlog.init = true;
    lsmlog.numfail++;
    logerror(0);
    while (1);
  }



  if (!lsmlog.fail) {
    // 1.) Set the accelerometer range

    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

    // 2.) Set the magnetometer sensitivity

    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

    // 3.) Setup the gyroscope

    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  }


}


/**************************************************************************/

void loop()
{
  get_GPS_data();
  get_IMU_data();

  //send the telemetry to the receiver
  sendData();

  if (progstart) {
    progstart = false;
  }

}

/**************************************************************************/

void get_IMU_data() {


  //Adafruit 9-DOF

  lsm.read();

  /* Get a new sensor event */
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t gyro_event;
  sensors_event_t temp_event;
  sensors_event_t bmp_event;
  sensors_vec_t orientation;

  //lsm.getEvent(&accel_event, &mag_event, &gyro_event, &temp_event);

  if (lsmlog.numfail < 5) {


    if (!lsmlog.fail && !lsmlog.init) {

      //Serial.println("lsm IMU");
      unsigned long lsm_ev_start = millis();
      while (!lsm.getEvent(&accel_event, &mag_event, &gyro_event, &temp_event)) {
        if (millis() - lsm_ev_start > 5) {//takes 2-3ms to read
          lsmlog.fail = true;
          lsmlog.numfail++;
          logerror(0);
          break;
        }
      }
    }
    else {
      lsmlog.fail = false;
    }


    if (lsmlog.fail) {
      lsmlog.init = true;
    }
    else {
      if (lsmlog.init) {

        lsm.begin();
        delay(5);
        lsm.read();
        lsm.getEvent(&accel_event, &mag_event, &gyro_event, &temp_event);

        lsmlog.fail = false;
        lsmlog.init = false;
      }
      else {
        //Serial.println("new lsm values...");

        data.accelx = accel_event.acceleration.x;
        data.accely = accel_event.acceleration.y;
        data.accelz = accel_event.acceleration.z;

        data.gyrox = gyro_event.gyro.x;
        data.gyroy = gyro_event.gyro.y;
        data.gyroz = gyro_event.gyro.z;

        data.magx = mag_event.magnetic.x;
        data.magy = mag_event.magnetic.y;
        data.magz = mag_event.magnetic.z;

        while (!lsm.fusionGetOrientation(&accel_event, &mag_event, &orientation));

        data.roll = orientation.roll;
        data.pitch = orientation.pitch;
        data.yaw = orientation.heading;

      }
    }
  }
  else {
    Serial.println("LSM IMU has failed 5 times");
  }


  if (bmplog.numfail < 5) {

    if (!bmplog.fail && !bmplog.init) {

      unsigned long bmpstart = millis();
      while (!bmp.getEvent(&bmp_event)) {
        if (millis() - bmpstart > 40) {
          //Serial.println("bmp failure");
          bmplog.fail = true;
          bmplog.numfail++;
          logerror(1);
          break;
        }
      }
    }
    else {
      bmplog.fail = false;
    }

    if (bmplog.fail) {
      bmplog.init = true;
    }
    else {
      if (bmplog.init) {
        bmplog.fail = false;
        bmplog.init = false;
        bmp.begin(BMP085_MODE_ULTRALOWPOWER);
      }
      else {
        bmp.getTemperature(&data.temperature);


        data.pressure = bmp_event.pressure;

        if (!gps.altitude.isValid()) {
          data.altitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure);
        }

      }
    }
  }
  else {
    Serial.println("bmp has failed more than 5 times");
  }


}

/**************************************************************************/
void get_GPS_data() {

  //GPS
  bool newData = false;

  long now = millis();
  
  // read GPS sensor every second
  if (now - lastMsg > 1000) {
    lastMsg = now;

    // parse GPS data and report some key values
    while (Serial2.available() > 0)
      if (gps.encode(Serial2.read())) {
        newData = true;
        //Serial.println("NEW GPS DATA");
      }

    //update latitude, longitude, and altitude
    if (newData) {
      //update location
      if (gps.location.isValid()) {
        data.latitude = gps.location.lat();
        data.longitude = gps.location.lng();
      }
      
      //update altitude
      if (gps.altitude.isValid()) {
        data.altitude = gps.altitude.meters();
      }

      //update time
      if (gps.time.isValid()) {
        mytime = gps.time.value();
      }

    }

    //timeout occured while reading GPS
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      //Serial.println("No GPS detected: check wiring.");
      logerror(3);
    }

  }


}

/**************************************************************************/
void sendData() {

  data.packet++;

  //Serial.println();
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.ID);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();//6

  Serial.print(data.packet);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();//6

  Serial.print(data.latitude, 4);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();//5

  Serial.print(data.longitude, 4);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();//5

  Serial.print(data.roll);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();//5

  Serial.print(data.pitch);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();//5

  Serial.print(data.yaw);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();//5

  Serial.print(data.accelx);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();//5

  Serial.print(data.accely);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();//5


  Serial.print(data.accelz);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  delay(5);

  Serial.print(data.magx);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.magy);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.magz);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.gyrox);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.gyroy);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.gyroz);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.temperature);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.pressure);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.altitude);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  Serial.print(data.endID);
  driver.waitPacketSent();
  driver.waitAuxHigh();
  Serial.print(",");
  driver.waitPacketSent();
  driver.waitAuxHigh();

  if (sd) {
    log(0, (float) data.packet);
    log(0, (float) mytime);

    log(4, data.latitude);
    log(4, data.longitude);

    log(2, data.roll);
    log(2, data.pitch);
    log(2, data.yaw);

    log(2, data.gyrox);
    log(2, data.gyroy);
    log(2, data.gyroz);

    log(2, data.accelx);
    log(2, data.accely);
    log(2, data.accelz);

    log(2, data.magx);
    log(2, data.magy);
    log(2, data.magz);

    log(2, data.altitude);
    log(2, data.temperature);
    log(2, data.pressure);
  }
}


/**************************************************************************/

void logerror(int type) {

  switch (type) {
    case 0:
      appendFile(SD, "/errorlog.txt", "000:LSM9DS1 FAILURE");
      appendFile(SD, "/log.txt", "\n ERROR 0 DETECTED \n");
      break;
    case 1:
      appendFile(SD, "/errorlog.txt", "001:BMP180 FAILURE");
      appendFile(SD, "/log.txt", "\n ERROR 1 DETECTED \n");
      break;
    case 2:
      appendFile(SD, "/errorlog.txt", "011:GPS FAILURE");
      appendFile(SD, "/log.txt", "\n ERROR 2 DETECTED \n");
      break;
    default:
      appendFile(SD, "/errorlog.txt", "100:UNKNOWN ERROR");
      appendFile(SD, "/log.txt", "\n ERROR 3 DETECTED \n");
      break;
  }

}

/**************************************************************************/
void log(int dec, float data) {

  unsigned long sdTime = millis();
  if (progstart) {
    writeFile(SD, "/log.txt", "Rocket Telemetry Log: \n");
    progstart = false;
  }

  if ((logcounter % 19) == 0) appendFile(SD, "/log.txt", "\n");

  dtostrf(data, 7, dec, outstr);
  appendFile(SD, "/log.txt", outstr);
  appendFile(SD, "/log.txt", "   ");

  if ((millis() - sdTime) >= 1800) {
    sd = false;
  }

  logcounter++;
}

/**************************************************************************/
void readFile(fs::FS & fs,
              const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS & fs,
               const char * path,
               const char * message) {
  //Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    //Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

/**************************************************************************/
void appendFile(fs::FS & fs,
                const char * path,
                const char * message) {
  //Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void deleteFile(fs::FS & fs,
                const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}
