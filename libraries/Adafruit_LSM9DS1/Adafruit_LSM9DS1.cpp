/***************************************************************************
  This is a library for the LSM9DS1 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM9DS1 Breakouts

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <Adafruit_LSM9DS1.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

void Adafruit_LSM9DS1::initI2C( TwoWire* wireBus, int32_t sensorID ) {
    _i2c = true;
    _wire = wireBus;
    _lsm9dso_sensorid_accel = sensorID + 1;
    _lsm9dso_sensorid_mag = sensorID + 2;
    _lsm9dso_sensorid_gyro = sensorID + 3;
    _lsm9dso_sensorid_temp = sensorID + 4;
    _accelSensor = Sensor(this, &Adafruit_LSM9DS1::readAccel, &Adafruit_LSM9DS1::getAccelEvent, &Adafruit_LSM9DS1::getAccelSensor);
    _magSensor   = Sensor(this, &Adafruit_LSM9DS1::readMag,   &Adafruit_LSM9DS1::getMagEvent,   &Adafruit_LSM9DS1::getMagSensor);
    _gyroSensor  = Sensor(this, &Adafruit_LSM9DS1::readGyro,  &Adafruit_LSM9DS1::getGyroEvent,  &Adafruit_LSM9DS1::getGyroSensor);
    _tempSensor  = Sensor(this, &Adafruit_LSM9DS1::readTemp,  &Adafruit_LSM9DS1::getTempEvent,  &Adafruit_LSM9DS1::getTempSensor);
}


// default
Adafruit_LSM9DS1::Adafruit_LSM9DS1( int32_t sensorID ) {
    initI2C(&Wire, sensorID);
}

Adafruit_LSM9DS1::Adafruit_LSM9DS1( TwoWire* wireBus, int32_t sensorID ) {
    initI2C(wireBus, sensorID);
}

Adafruit_LSM9DS1::Adafruit_LSM9DS1(int8_t xgcs, int8_t mcs, int32_t sensorID ) {
  _i2c = false;
  // hardware SPI!
  _csm = mcs;
  _csxg = xgcs;
  _mosi = _miso = _clk = -1;
  _lsm9dso_sensorid_accel = sensorID + 1;
  _lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
  _accelSensor = Sensor(this, &Adafruit_LSM9DS1::readAccel, &Adafruit_LSM9DS1::getAccelEvent, &Adafruit_LSM9DS1::getAccelSensor);
  _magSensor   = Sensor(this, &Adafruit_LSM9DS1::readMag,   &Adafruit_LSM9DS1::getMagEvent,   &Adafruit_LSM9DS1::getMagSensor);
  _gyroSensor  = Sensor(this, &Adafruit_LSM9DS1::readGyro,  &Adafruit_LSM9DS1::getGyroEvent,  &Adafruit_LSM9DS1::getGyroSensor);
  _tempSensor  = Sensor(this, &Adafruit_LSM9DS1::readTemp,  &Adafruit_LSM9DS1::getTempEvent,  &Adafruit_LSM9DS1::getTempSensor);
}

Adafruit_LSM9DS1::Adafruit_LSM9DS1(int8_t sclk, int8_t smiso, int8_t smosi, int8_t xgcs, int8_t mcs, int32_t sensorID ) {
  _i2c = false;
  // software SPI!
  _csm = mcs;
  _csxg = xgcs;
  _mosi = smosi;
  _miso = smiso;
  _clk = sclk;
  _lsm9dso_sensorid_accel = sensorID + 1;
  _lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
  _accelSensor = Sensor(this, &Adafruit_LSM9DS1::readAccel, &Adafruit_LSM9DS1::getAccelEvent, &Adafruit_LSM9DS1::getAccelSensor);
  _magSensor   = Sensor(this, &Adafruit_LSM9DS1::readMag,   &Adafruit_LSM9DS1::getMagEvent,   &Adafruit_LSM9DS1::getMagSensor);
  _gyroSensor  = Sensor(this, &Adafruit_LSM9DS1::readGyro,  &Adafruit_LSM9DS1::getGyroEvent,  &Adafruit_LSM9DS1::getGyroSensor);
  _tempSensor  = Sensor(this, &Adafruit_LSM9DS1::readTemp,  &Adafruit_LSM9DS1::getTempEvent,  &Adafruit_LSM9DS1::getTempSensor);
}

bool Adafruit_LSM9DS1::begin()
{
  if (_i2c) {
    _wire->begin();
  } else if (_clk == -1) {
    // Hardware SPI
    pinMode(_csxg, OUTPUT);
    pinMode(_csm, OUTPUT);
    digitalWrite(_csxg, HIGH);
    digitalWrite(_csm, HIGH);
    SPI.begin();
  } else {
    //Serial.println("softSPI");
    // Sofware SPI
    pinMode(_clk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    pinMode(_csxg, OUTPUT);
    pinMode(_csm, OUTPUT);
    digitalWrite(_csxg, HIGH);
    digitalWrite(_csm, HIGH);
    digitalWrite(_clk, HIGH);
  }


  // soft reset & reboot accel/gyro
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG8, 0x05);
  // soft reset & reboot magnetometer
  write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C);

  delay(10);


  /*
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("XG $"); Serial.print(i, HEX); Serial.print(" = 0x");
    Serial.println(read8(XGTYPE, i), HEX);
  }
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("M $"); Serial.print(i, HEX); Serial.print(" = 0x");
    Serial.println(read8(MAGTYPE, i), HEX);
  }
  */

  uint8_t id = read8(XGTYPE, LSM9DS1_REGISTER_WHO_AM_I_XG);
  //Serial.print ("XG whoami: 0x"); Serial.println(id, HEX);
  if (id != LSM9DS1_XG_ID)
    return false;

  id = read8(MAGTYPE, LSM9DS1_REGISTER_WHO_AM_I_M);
  //Serial.print ("MAG whoami: 0x"); Serial.println(id, HEX);
  if (id != LSM9DS1_MAG_ID)
    return false;

  // enable gyro continuous
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0); // on XYZ

  // Enable the accelerometer continous
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38); // enable X Y and Z axis
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0); // 1 KHz out data rate, BW set by ODR, 408Hz anti-aliasing


  // enable mag continuous
  //write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG1_M, 0xFC); // high perf XY, 80 Hz ODR
  write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG3_M, 0x00); // continuous mode
  //write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG4_M, 0x0C); // high perf Z mode



  // Set default ranges for the various sensors
  setupAccel(LSM9DS1_ACCELRANGE_2G);
  setupMag(LSM9DS1_MAGGAIN_4GAUSS);
  setupGyro(LSM9DS1_GYROSCALE_245DPS);

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM9DS1::read()
{
  /* Read all the sensors. */
  readAccel();
  readMag();
  readGyro();
  readTemp();
}

void Adafruit_LSM9DS1::readAccel() {
  // Read the accelerometer
  byte buffer[6];
  readBuffer(XGTYPE,
       0x80 | LSM9DS1_REGISTER_OUT_X_L_XL,
       6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;
  accelData.x = xhi;
  accelData.y = yhi;
  accelData.z = zhi;
}

void Adafruit_LSM9DS1::readMag() {
  // Read the magnetometer
  byte buffer[6];
  readBuffer(MAGTYPE,
       0x80 | LSM9DS1_REGISTER_OUT_X_L_M,
       6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;
  magData.x = xhi;
  magData.y = yhi;
  magData.z = zhi;
}

void Adafruit_LSM9DS1::readGyro() {
  // Read gyro
  byte buffer[6];
  readBuffer(XGTYPE,
       0x80 | LSM9DS1_REGISTER_OUT_X_L_G,
       6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;

  gyroData.x = xhi;
  gyroData.y = yhi;
  gyroData.z = zhi;
}

void Adafruit_LSM9DS1::readTemp() {
  // Read temp sensor
  byte buffer[2];
  readBuffer(XGTYPE,
       0x80 | LSM9DS1_REGISTER_TEMP_OUT_L,
       2, buffer);
  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];

  xhi <<= 8; xhi |= xlo;

  // Shift values to create properly formed integer (low byte first)
  temperature = xhi;
}

void Adafruit_LSM9DS1::setupAccel ( lsm9ds1AccelRange_t range )
{
  uint8_t reg = read8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG6_XL);
  reg &= ~(0b00011000);
  reg |= range;
  //Serial.println("set range: ");
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG6_XL, reg );

  switch (range)
  {
    case LSM9DS1_ACCELRANGE_2G:
      _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
      break;
    case LSM9DS1_ACCELRANGE_4G:
      _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
      break;
    case LSM9DS1_ACCELRANGE_8G:
      _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
      break;
    case LSM9DS1_ACCELRANGE_16G:
      _accel_mg_lsb =LSM9DS1_ACCEL_MG_LSB_16G;
      break;
  }
}

void Adafruit_LSM9DS1::setupMag ( lsm9ds1MagGain_t gain )
{
  uint8_t reg = read8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG2_M);
  reg &= ~(0b01100000);
  reg |= gain;
  write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG2_M, reg );

  switch(gain)
  {
    case LSM9DS1_MAGGAIN_4GAUSS:
      _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_4GAUSS;
      break;
    case LSM9DS1_MAGGAIN_8GAUSS:
      _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_8GAUSS;
      break;
    case LSM9DS1_MAGGAIN_12GAUSS:
      _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_12GAUSS;
      break;
    case LSM9DS1_MAGGAIN_16GAUSS:
      _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_16GAUSS;
      break;
  }
}

void Adafruit_LSM9DS1::setupGyro ( lsm9ds1GyroScale_t scale )
{
  uint8_t reg = read8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG1_G);
  reg &= ~(0b00011000);
  reg |= scale;
  write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG1_G, reg );

  switch(scale)
  {
    case LSM9DS1_GYROSCALE_245DPS:
      _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_245DPS;
      break;
    case LSM9DS1_GYROSCALE_500DPS:
      _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_500DPS;
      break;
    case LSM9DS1_GYROSCALE_2000DPS:
      _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_2000DPS;
      break;
  }
}


/***************************************************************************
 UNIFIED SENSOR FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Gets the most recent accel sensor event
*/
/**************************************************************************/
bool Adafruit_LSM9DS1::getEvent(sensors_event_t *accelEvent,
                                sensors_event_t *magEvent,
                                sensors_event_t *gyroEvent,
                                sensors_event_t *tempEvent )
{
  /* Grab new sensor reading and timestamp. */
  read();
  uint32_t timestamp = millis();

  /* Update appropriate sensor events. */
  if (accelEvent) getAccelEvent(accelEvent, timestamp);
  if (magEvent)   getMagEvent(magEvent, timestamp);
  if (gyroEvent)  getGyroEvent(gyroEvent, timestamp);
  if (tempEvent)  getTempEvent(tempEvent, timestamp);

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getSensor(sensor_t *accel, sensor_t *mag,
                                 sensor_t *gyro, sensor_t *temp )
{
  /* Update appropriate sensor metadata. */
  if (accel) getAccelSensor(accel);
  if (mag)   getMagSensor(mag);
  if (gyro)  getGyroSensor(gyro);
  if (temp)  getTempSensor(temp);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM9DS1::write8(boolean type, byte reg, byte value)
{
  byte address, _cs;

  if (type == MAGTYPE) {
    address = LSM9DS1_ADDRESS_MAG;
    _cs = _csm;
  } else {
    address = LSM9DS1_ADDRESS_ACCELGYRO;
    _cs = _csxg;
  }
  if (_i2c) {
    _wire->beginTransmission(address);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
    /*
    Serial.print("0x"); Serial.print(address, HEX);
    Serial.print(" $"); Serial.print(reg, HEX); Serial.print(" = ");
    Serial.println(value, HEX);
    */
  } else {
    digitalWrite(_cs, LOW);
    if (_clk == -1)     // hardware SPI
      SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
    // set address
    spixfer(reg & 0x7F); // write data
    spixfer(value);
    if (_clk == -1)     // hardware SPI
      SPI.endTransaction();
    digitalWrite(_cs, HIGH);

  }
}

byte Adafruit_LSM9DS1::read8(boolean type, byte reg)
{
  uint8_t value;

  readBuffer(type, reg, 1, &value);

  return value;
}

byte Adafruit_LSM9DS1::readBuffer(boolean type, byte reg, byte len, uint8_t *buffer)
{
  byte address, _cs;

  if (type == MAGTYPE) {
    address = LSM9DS1_ADDRESS_MAG;
    _cs = _csm;
  } else {
    address = LSM9DS1_ADDRESS_ACCELGYRO;
    _cs = _csxg;
  }

  if (_i2c) {
    _wire->beginTransmission(address);
    _wire->write(reg);
    _wire->endTransmission();
    if (_wire->requestFrom(address, (byte)len) != len) {
      return 0;
    }

    /*
      Serial.print("0x"); Serial.print(address, HEX);
      Serial.print(" $"); Serial.print(reg, HEX); Serial.print(": ");
    */

    for (uint8_t i=0; i<len; i++) {
      buffer[i] = _wire->read();
      //Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    //Serial.println();

  } else {
    if (_clk == -1)     // hardware SPI
      SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
    else
      digitalWrite(_clk, HIGH);
    // set address

    digitalWrite(_cs, LOW);

    if(type == MAGTYPE)
      reg |= 0x40;

    spixfer(reg | 0x80 ); // readdata
    for (uint8_t i=0; i<len; i++) {
      buffer[i] = spixfer(0);
    }
    if (_clk == -1)     // hardware SPI
      SPI.endTransaction();
    else
      digitalWrite(_clk, HIGH);
    digitalWrite(_cs, HIGH);
  }

  return len;
}

uint8_t Adafruit_LSM9DS1::spixfer(uint8_t data) {

  if (_clk == -1) {
      //Serial.println("Hardware SPI");
      return SPI.transfer(data);
  } else {
    //Serial.println("Software SPI");
    uint8_t reply = 0;
    for (int i=7; i>=0; i--) {
      reply <<= 1;
      digitalWrite(_clk, LOW);
      digitalWrite(_mosi, data & (1<<i));
      digitalWrite(_clk, HIGH);
      if (digitalRead(_miso))
	reply |= 1;
    }
    return reply;
  }
}

void Adafruit_LSM9DS1::getAccelEvent(sensors_event_t* event, uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_accel;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = timestamp;
  event->acceleration.x = accelData.x * _accel_mg_lsb;
  event->acceleration.x /= 1000;
  event->acceleration.x *= SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = accelData.y * _accel_mg_lsb;
  event->acceleration.y /= 1000;
  event->acceleration.y *= SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = accelData.z * _accel_mg_lsb;
  event->acceleration.z /= 1000;
  event->acceleration.z *= SENSORS_GRAVITY_STANDARD;
}

void Adafruit_LSM9DS1::getMagEvent(sensors_event_t* event, uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_mag;
  event->type      = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = timestamp;
  event->magnetic.x = magData.x * _mag_mgauss_lsb;
  event->magnetic.x /= 1000;
  event->magnetic.y = magData.y * _mag_mgauss_lsb;
  event->magnetic.y /= 1000;
  event->magnetic.z = magData.z * _mag_mgauss_lsb;
  event->magnetic.z /= 1000;
}

void Adafruit_LSM9DS1::getGyroEvent(sensors_event_t* event, uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_accel;
  event->type      = SENSOR_TYPE_GYROSCOPE;
  event->timestamp = timestamp;
  event->gyro.x = gyroData.x * _gyro_dps_digit;
  event->gyro.y = gyroData.y * _gyro_dps_digit;
  event->gyro.z = gyroData.z * _gyro_dps_digit;
}

void Adafruit_LSM9DS1::getTempEvent(sensors_event_t* event, uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_temp;
  event->type      = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = timestamp;
  // This is just a guess since the staring point (21C here) isn't documented :(
  event->temperature = 21.0 + (float)temperature/8;
  //event->temperature /= LSM9DS1_TEMP_LSB_DEGREE_CELSIUS;
}

void Adafruit_LSM9DS1::getAccelSensor(sensor_t* sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy (sensor->name, "LSM9DS1_A", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _lsm9dso_sensorid_accel;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0;  // ToDo
  sensor->min_value   = 0.0;  // ToDo
  sensor->resolution  = 0.0;  // ToDo
}

void Adafruit_LSM9DS1::getMagSensor(sensor_t* sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy (sensor->name, "LSM9DS1_M", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _lsm9dso_sensorid_mag;
  sensor->type        = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0;  // ToDo
  sensor->min_value   = 0.0;  // ToDo
  sensor->resolution  = 0.0;  // ToDo
}

void Adafruit_LSM9DS1::getGyroSensor(sensor_t* sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy (sensor->name, "LSM9DS1_G", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _lsm9dso_sensorid_gyro;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0;  // ToDo
  sensor->min_value   = 0.0;  // ToDo
  sensor->resolution  = 0.0;  // ToDo
}

void Adafruit_LSM9DS1::getTempSensor(sensor_t* sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy (sensor->name, "LSM9DS1_T", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _lsm9dso_sensorid_temp;
  sensor->type        = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0;  // ToDo
  sensor->min_value   = 0.0;  // ToDo
  sensor->resolution  = 0.0;  // ToDo
}

bool Adafruit_LSM9DS1::accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
  /* Make sure the input is valid, not null, etc. */
  if (event == NULL) return false;
  if (orientation == NULL) return false;

  float t_pitch;
  float t_roll;
  float t_heading;
  float signOfZ = event->acceleration.z >= 0 ? 1.0F : -1.0F;

  /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -90<=roll<=90    */
  /* roll is positive and increasing when moving downward                                     */
  /*                                                                                          */
  /*                                 y                                                        */
  /*             roll = atan(-----------------)                                               */
  /*                          sqrt(x^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_roll = event->acceleration.x * event->acceleration.x + event->acceleration.z * event->acceleration.z;
  orientation->roll = (float)atan2(event->acceleration.y, sqrt(t_roll)) * 180 / PI;

  /* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)     */
  /* pitch is positive and increasing when moving upwards                                     */
  /*                                                                                          */
  /*                                 x                                                        */
  /*            pitch = atan(-----------------)                                               */
  /*                          sqrt(y^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_pitch = event->acceleration.y * event->acceleration.y + event->acceleration.z * event->acceleration.z;
  orientation->pitch = (float)atan2(event->acceleration.x, signOfZ * sqrt(t_pitch)) * 180 / PI;

  return true;
}

/**************************************************************************/
/*!
    @brief  Utilize the sensor data from an accelerometer to compensate
            the magnetic sensor measurements when the sensor is tilted
            (the pitch and roll angles are not equal 0°)

    @param  axis          The given axis (SENSOR_AXIS_X/Y/Z) that is
                          parallel to the gravity of the Earth

    @param  mag_event     The raw magnetometer data to adjust for tilt

    @param  accel_event   The accelerometer event data to use to determine
                          the tilt when compensating the mag_event values

    @code

    // Perform tilt compensation with matching accelerometer data
    sensors_event_t accel_event;
    error = lsm303accelGetSensorEvent(&accel_event);
    if (!error)
    {
      magTiltCompensation(SENSOR_AXIS_Z, &mag_event, &accel_event);
    }

    @endcode
*/
/**************************************************************************/
bool Adafruit_LSM9DS1::magTiltCompensation(sensors_axis_t axis, sensors_event_t *mag_event, sensors_event_t *accel_event)
{
  /* Make sure the input is valid, not null, etc. */
  if (mag_event == NULL) return false;
  if (accel_event == NULL) return false;

  float accel_X, accel_Y, accel_Z;
  float *mag_X, *mag_Y, *mag_Z;

  switch (axis)
  {
    case SENSOR_AXIS_X:
      /* The X-axis is parallel to the gravity */
      accel_X = accel_event->acceleration.y;
      accel_Y = accel_event->acceleration.z;
      accel_Z = accel_event->acceleration.x;
      mag_X = &(mag_event->magnetic.y);
      mag_Y = &(mag_event->magnetic.z);
      mag_Z = &(mag_event->magnetic.x);
      break;

    case SENSOR_AXIS_Y:
      /* The Y-axis is parallel to the gravity */
      accel_X = accel_event->acceleration.z;
      accel_Y = accel_event->acceleration.x;
      accel_Z = accel_event->acceleration.y;
      mag_X = &(mag_event->magnetic.z);
      mag_Y = &(mag_event->magnetic.x);
      mag_Z = &(mag_event->magnetic.y);
      break;

    case SENSOR_AXIS_Z:
      /* The Z-axis is parallel to the gravity */
      accel_X = accel_event->acceleration.x;
      accel_Y = accel_event->acceleration.y;
      accel_Z = accel_event->acceleration.z;
      mag_X = &(mag_event->magnetic.x);
      mag_Y = &(mag_event->magnetic.y);
      mag_Z = &(mag_event->magnetic.z);
      break;

    default:
      return false;
  }

  float t_roll = accel_X * accel_X + accel_Z * accel_Z;
  float rollRadians = (float)atan2(accel_Y, sqrt(t_roll));

  float t_pitch = accel_Y * accel_Y + accel_Z * accel_Z;
  float pitchRadians = (float)atan2(accel_X, sqrt(t_pitch));

  float cosRoll = (float)cos(rollRadians);
  float sinRoll = (float)sin(rollRadians);
  float cosPitch = (float)cos(-1*pitchRadians);
  float sinPitch = (float)sin(-1*pitchRadians);

  /* The tilt compensation algorithm                            */
  /* Xh = X.cosPitch + Z.sinPitch                               */
  /* Yh = X.sinRoll.sinPitch + Y.cosRoll - Z.sinRoll.cosPitch   */
  float raw_mag_X = *mag_X;
  float raw_mag_Y = *mag_Y;
  float raw_mag_Z = *mag_Z;
  *mag_X = (raw_mag_X) * cosPitch + (raw_mag_Z) * sinPitch;
  *mag_Y = (raw_mag_X) * sinRoll * sinPitch + (raw_mag_Y) * cosRoll - (raw_mag_Z) * sinRoll * cosPitch;

  return true;
}

/**************************************************************************/
/*!
    @brief  Populates the .heading fields in the sensors_vec_t
            struct with the right angular data (0-359°)

            Heading increases when measuring clockwise

    @param  axis          The given axis (SENSOR_AXIS_X/Y/Z)

    @param  event         The raw magnetometer sensor data to use when
                          calculating out heading

    @param  orientation   The sensors_vec_t object where we will
                          assign an 'orientation.heading' value

    @code

    magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);

    @endcode
*/
/**************************************************************************/
bool Adafruit_LSM9DS1::magGetOrientation(sensors_axis_t axis, sensors_event_t *event, sensors_vec_t *orientation)
{
  /* Make sure the input is valid, not null, etc. */
  if (event == NULL) return false;
  if (orientation == NULL) return false;

  switch (axis)
  {
    case SENSOR_AXIS_X:
      /* Sensor rotates around X-axis                                                                 */
      /* "heading" is the angle between the 'Y axis' and magnetic north on the horizontal plane (Oyz) */
      /* heading = atan(Mz / My)                                                                      */
      orientation->heading = (float)atan2(event->magnetic.z, event->magnetic.y) * 180 / PI;
      break;

    case SENSOR_AXIS_Y:
      /* Sensor rotates around Y-axis                                                                 */
      /* "heading" is the angle between the 'Z axis' and magnetic north on the horizontal plane (Ozx) */
      /* heading = atan(Mx / Mz)                                                                      */
      orientation->heading = (float)atan2(event->magnetic.x, event->magnetic.z) * 180 / PI;
      break;

    case SENSOR_AXIS_Z:
      /* Sensor rotates around Z-axis                                                                 */
      /* "heading" is the angle between the 'X axis' and magnetic north on the horizontal plane (Oxy) */
      /* heading = atan(My / Mx)                                                                      */
      orientation->heading = (float)atan2(event->magnetic.y, event->magnetic.x) * 180 / PI;
      break;

    default:
      return false;
  }

  /* Normalize to 0-359° */
  if (orientation->heading < 0)
  {
    orientation->heading = 360 + orientation->heading;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Populates the .roll/.pitch/.heading fields in the sensors_vec_t
            struct with the right angular data (in degree).

            The starting position is set by placing the object flat and
            pointing northwards (Z-axis pointing upward and X-axis pointing
            northwards).

            The orientation of the object can be modeled as resulting from
            3 consecutive rotations in turn: heading (Z-axis), pitch (Y-axis),
            and roll (X-axis) applied to the starting position.


    @param  accel_event   The sensors_event_t variable containing the
                          data from the accelerometer

    @param  mag_event     The sensors_event_t variable containing the
                          data from the magnetometer

    @param  orientation   The sensors_vec_t object that will have it's
                          .roll, .pitch and .heading fields populated
*/
/**************************************************************************/
bool Adafruit_LSM9DS1::fusionGetOrientation(sensors_event_t *accel_event, sensors_event_t *mag_event, sensors_vec_t *orientation)
{
  /* Make sure the input is valid, not null, etc. */
  if ( accel_event  == NULL) return false;
  if ( mag_event    == NULL) return false;
  if ( orientation  == NULL) return false;

  float const PI_F = 3.14159265F;

  /* roll: Rotation around the X-axis. -180 <= roll <= 180                                          */
  /* a positive roll angle is defined to be a clockwise rotation about the positive X-axis          */
  /*                                                                                                */
  /*                    y                                                                           */
  /*      roll = atan2(---)                                                                         */
  /*                    z                                                                           */
  /*                                                                                                */
  /* where:  y, z are returned value from accelerometer sensor                                      */
  orientation->roll = (float)atan2(accel_event->acceleration.y, accel_event->acceleration.z);

  /* pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         */
  /* a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         */
  /*                                                                                                */
  /*                                 -x                                                             */
  /*      pitch = atan(-------------------------------)                                             */
  /*                    y * sin(roll) + z * cos(roll)                                               */
  /*                                                                                                */
  /* where:  x, y, z are returned value from accelerometer sensor                                   */
  if (accel_event->acceleration.y * sin(orientation->roll) + accel_event->acceleration.z * cos(orientation->roll) == 0)
    orientation->pitch = accel_event->acceleration.x > 0 ? (PI_F / 2) : (-PI_F / 2);
  else
    orientation->pitch = (float)atan(-accel_event->acceleration.x / (accel_event->acceleration.y * sin(orientation->roll) + \
                                                                     accel_event->acceleration.z * cos(orientation->roll)));

  /* heading: Rotation around the Z-axis. -180 <= roll <= 180                                       */
  /* a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       */
  /*                                                                                                */
  /*                                       z * sin(roll) - y * cos(roll)                            */
  /*   heading = atan2(--------------------------------------------------------------------------)  */
  /*                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))   */
  /*                                                                                                */
  /* where:  x, y, z are returned value from magnetometer sensor                                    */
  orientation->heading = (float)atan2(mag_event->magnetic.z * sin(orientation->roll) - mag_event->magnetic.y * cos(orientation->roll), \
                                      mag_event->magnetic.x * cos(orientation->pitch) + \
                                      mag_event->magnetic.y * sin(orientation->pitch) * sin(orientation->roll) + \
                                      mag_event->magnetic.z * sin(orientation->pitch) * cos(orientation->roll));


  /* Convert angular data to degree */
  orientation->roll = orientation->roll * 180 / PI_F;
  orientation->pitch = orientation->pitch * 180 / PI_F;
  orientation->heading = orientation->heading * 180 / PI_F;

  return true;
}
/*
#define DECLINATION -10 //change for specific region (D.C.)

bool Adafruit_LSM9DS1::fusionGetOrientation(sensors_event_t *accel_event, sensors_event_t *mag_event, sensors_vec_t *orientation)
{
  /* Make sure the input is valid, not null, etc. *//*
  if ( accel_event  == NULL) return false;
  if ( mag_event    == NULL) return false;
  if ( orientation  == NULL) return false;

  float const PI_F = 3.14159265F;

  orientation->roll = atan2(accel_event->acceleration.y, accel_event->acceleration.z);
  orientation->pitch = atan2(-accel_event->acceleration.x, sqrt(accel_event->acceleration.y * accel_event->acceleration.y + accel_event->acceleration.z * accel_event->acceleration.z));


if (mag_event->magnetic.y == 0)
  orientation->heading = (mag_event->magnetic.x < 0) ? PI_F : 0;
else
  orientation->heading = atan2(mag_event->magnetic.x, mag_event->magnetic.y);

orientation->heading -= DECLINATION * PI_F / 180;

if (orientation->heading > PI_F) orientation->heading -= (2 * PI_F);
else if (orientation->heading < -PI_F) orientation->heading += (2 * PI_F);

// Convert everything from radians to degrees:
orientation->heading *= 180.0 / PI_F;
orientation->pitch *= 180.0 / PI_F;
orientation->roll  *= 180.0 / PI_F;

  return true;
}*/
