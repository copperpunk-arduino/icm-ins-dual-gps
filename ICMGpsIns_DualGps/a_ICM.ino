void icmSetup()
{
  Wire.begin();
  Wire.setClock(400000);

  struct inv_icm20948_serif icm20948_serif;
  icm20948_serif.context = 0; /* no need */
  icm20948_serif.read_reg = idd_io_hal_read_reg;
  icm20948_serif.write_reg = idd_io_hal_write_reg;
  icm20948_serif.max_read = 1024 * 16;  /* maximum number of bytes allowed per serial read */
  icm20948_serif.max_write = 1024 * 16; /* maximum number of bytes allowed per serial write */

  icm20948_serif.is_spi = false;

  icm_device_.base_state.serial_interface = SERIAL_INTERFACE_I2C;

  inv_icm20948_reset_states(&icm_device_, &icm20948_serif);

  int rc = icm20948_sensor_setup();

  if (icm_device_.selftest_done && !icm_device_.offset_done)
  {
    // If we've run selftes and not already set the offset.
    inv_icm20948_set_offset(&icm_device_, unscaled_bias_);
    icm_device_.offset_done = 1;
  }
  //enable sensors
  rc = inv_icm20948_enable_sensor(&icm_device_, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), 1);
  rc = inv_icm20948_enable_sensor(&icm_device_, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), 1);
  rc = inv_icm20948_enable_sensor(&icm_device_, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), 1);
  rc = inv_icm20948_set_sensor_period(&icm_device_, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), kImuIntervalMs);
  rc = inv_icm20948_set_sensor_period(&icm_device_, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), kImuIntervalMs);
  rc = inv_icm20948_set_sensor_period(&icm_device_, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), kImuIntervalMs);
}

static uint8_t icm20948_get_grv_accuracy(void)
{
  uint8_t accel_accuracy;
  uint8_t gyro_accuracy;

  accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
  gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
  return (min(accel_accuracy, gyro_accuracy));
}

void build_sensor_event_data(void *context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void *data, const void *arg)
{

  inv_sensor_event_t event;
  (void)context;
  uint8_t sensor_id = convert_to_generic_ids[sensortype];

  memset((void *)&event, 0, sizeof(event));
  event.sensor = sensor_id;
  event.timestamp = timestamp;
  switch (sensor_id)
  {
  case INV_SENSOR_TYPE_GYROSCOPE:
    memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
    gyro_rps_[0] = event.data.gyr.vect[0] * kDeg2Rad;
    gyro_rps_[1] = -event.data.gyr.vect[1] * kDeg2Rad;
    gyro_rps_[2] = -event.data.gyr.vect[2] * kDeg2Rad;

    break;
  case INV_SENSOR_TYPE_ACCELEROMETER:
    memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
    accel_body_mpss_[0] = -event.data.acc.vect[0] * kGravity;
    accel_body_mpss_[1] = event.data.acc.vect[1] * kGravity;
    accel_body_mpss_[2] = event.data.acc.vect[2] * kGravity;

    break;
  case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
    memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
    quat_[0] = event.data.quaternion.quat[0];
    quat_[1] = event.data.quaternion.quat[1];
    quat_[2] = -event.data.quaternion.quat[2];
    quat_[3] = -event.data.quaternion.quat[3];
    new_icm_data_ = true;
    break;
  default:
    return;
  }
}

void pollIcm()
{
  //  DebugPrint("poll: ");
  inv_icm20948_poll_sensor(&icm_device_, (void *)0, build_sensor_event_data);
}

/*
  Sleep implementation for ICM20948
*/
void inv_icm20948_sleep(int ms)
{
  delay(ms);
}

void inv_icm20948_sleep_us(int us)
{
  delayMicroseconds(us);
}

int i2c_master_write_register(uint8_t address, uint8_t reg, uint32_t len, const uint8_t *data)
{
  if (address != 0x69)
  {

    DebugPrint("Odd address:");
    DebugPrintln(address);
  }
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data, len);
  Wire.endTransmission();
  return 0;
}

int i2c_master_read_register(uint8_t address, uint8_t reg, uint32_t len, uint8_t *buff)
{
  if (address != 0x69)
  {

    DebugPrint("Odd read address:");
    DebugPrintln(address);
  }

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false); // Send repeated start

  uint32_t offset = 0;
  uint32_t num_received = Wire.requestFrom(address, len);
  if (num_received == len)
  {
    for (uint8_t i = 0; i < len; i++)
    {
      buff[i] = Wire.read();
    }
    return 0;
  }
  else
  {
    return -1;
  }
}

//---------------------------------------------------------------------
int idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
  return i2c_master_read_register(I2C_Address, reg, rlen, rbuffer);
}

//---------------------------------------------------------------------

int idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
  return i2c_master_write_register(I2C_Address, reg, wlen, wbuffer);
}

//---------------------------------------------------------------------
void icm20948_apply_mounting_matrix(void)
{
  int ii;
  for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++)
  {
    inv_icm20948_set_matrix(&icm_device_, cfg_mounting_matrix, (inv_icm20948_sensor)ii);
  }
}

//---------------------------------------------------------------------

static void icm20948_set_fsr(void)
{
  inv_icm20948_set_fsr(&icm_device_, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
  inv_icm20948_set_fsr(&icm_device_, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
  inv_icm20948_set_fsr(&icm_device_, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
}

//--------------------------------------------------------------------

int icm20948_sensor_setup(void)
{
  int rc;
  uint8_t i, whoami = 0xff;

  /*
    Just get the whoami
  */
  rc = inv_icm20948_get_whoami(&icm_device_, &whoami);
  DebugPrint("whoami = ");
  DebugPrintln(whoami);

  //delay(1000);

  /* Setup accel and gyro mounting matrix and associated angle for current board */
  inv_icm20948_init_matrix(&icm_device_);

  DebugPrint("dmp image size = ");
  DebugPrintln(sizeof(dmp3_image));
  rc = inv_icm20948_initialize(&icm_device_, dmp3_image, sizeof(dmp3_image));
  while (rc != 0)
  {
    delay(500);
    rc = inv_icm20948_initialize(&icm_device_, dmp3_image, sizeof(dmp3_image));
    DebugPrint("init got ");
    DebugPrintln(rc);
    //  INV_MSG(INV_MSG_LEVEL_ERROR, "Initialization failed. Error loading DMP3...");
    //    return rc;
  }

  icm20948_apply_mounting_matrix();

  icm20948_set_fsr();

  /* re-initialize base state structure */
  inv_icm20948_init_structure(&icm_device_);
  return 0;
} //sensor_setup

//---------------------------------------------------------------------

uint64_t inv_icm20948_get_time_us(void)
{
  return millis();
}

//---------------------------------------------------------------------

static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor)
{
  switch (sensor)
  {
  case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
    return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
  case INV_SENSOR_TYPE_RAW_GYROSCOPE:
    return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
  case INV_SENSOR_TYPE_ACCELEROMETER:
    return INV_ICM20948_SENSOR_ACCELEROMETER;
  case INV_SENSOR_TYPE_GYROSCOPE:
    return INV_ICM20948_SENSOR_GYROSCOPE;
  case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
    return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
  case INV_SENSOR_TYPE_ROTATION_VECTOR:
    return INV_ICM20948_SENSOR_ROTATION_VECTOR;
  default:
    return INV_ICM20948_SENSOR_MAX;
  } //switch
} //enum sensortyp_conversion
