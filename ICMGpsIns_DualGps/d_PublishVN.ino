#define field_bit(x) (1<<(x))

void PublishVnBinary1()
{
//  byte vn_binary1_buffer_[kVnBinary1Length];
//  vn_binary1_buffer_[0] = 0xFA;
//  vn_binary1_buffer_[1] = 1; // Group 1
  // Fields 1
  // 1: TimeGPS (8)
  // 3: YPR (12)
  // 5: Body rate (12)
  // 6: Position-LLA (24)
  // 7: Velocity-NED (12)
  //-----------------------
  // Fields 2
  // 10: MagPres (20)
  // 12: INS status (2)
  uint16_t fields_word = field_bit(1) | field_bit(3) | field_bit(5) | field_bit(6) | field_bit(7) | field_bit(8) | field_bit(12);// field_bit(10) | field_bit(12);
  byte fields_1 = fields_word & 0xFF;
  byte fields_2 = fields_word >> 8;
  vn_binary1_buffer_[2] = fields_1;
  vn_binary1_buffer_[3] = fields_2;
  // Sum of Data: 8 + 12 + 12 + 24 + 12 + 12 + 2 = 82
  // Message Length: 82 + 6 = 108
  int index = 4;
  long current_time_us = micros();
  long time_since_last_gps_update_us = current_time_us - system_time_prev_us_;
  uint64_t gps_time_ns = gps_fix_time_*(1e9) + (time_since_last_gps_update_us) * 1000 ;
  //DebugPrint("\nns: " + String(uint32_t((gps_time_ns>>32) & 0xFFFFFFFF)) + String(uint32_t(gps_time_ns & 0xFFFFFFFF)));
  float yaw_deg = attitude_[2] * RAD2DEG;
  float pitch_deg = attitude_[1] * RAD2DEG;
  float roll_deg = attitude_[0] * RAD2DEG;
  memcpy(&vn_binary1_buffer_[index], &gps_time_ns, 8); index += 8;
  memcpy(&vn_binary1_buffer_[index], &yaw_deg, 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &pitch_deg, 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &roll_deg, 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &gyro_float_[0], 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &gyro_float_[1], 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &gyro_float_[2], 4); index += 4;
  double lat1;
  double lon1;
  calculateLatLonFromDeltaXDeltaY(ekf_position_, lat1, lon1);
//  debugPrintln("lat/lon: " + String(lat1*RAD2DEG,7) + "/" + String(lon1*RAD2DEG,7));
  lat1 *= RAD2DEG;
  lon1 *= RAD2DEG;
  double alt_double = (double)ekf_position_[2];
  memcpy(&vn_binary1_buffer_[index], &lat1, 8); index += 8;
  memcpy(&vn_binary1_buffer_[index], &lon1, 8); index += 8;
  memcpy(&vn_binary1_buffer_[index], &alt_double, 8); index += 8;
  memcpy(&vn_binary1_buffer_[index], &ekf_velocity_[0], 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &ekf_velocity_[1], 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &ekf_velocity_[2], 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &accel_body_[0], 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &accel_body_[1], 4); index += 4;
  memcpy(&vn_binary1_buffer_[index], &accel_body_[2], 4); index += 4;
//  memcpy(&vn_binary1_buffer_[index], &imu_data_[6], 4); index += 4;
//  memcpy(&vn_binary1_buffer_[index], &imu_data_[7], 4); index += 4;
//  memcpy(&vn_binary1_buffer_[index], &imu_data_[8], 4); index += 4;
//  float temp_fake = 12.34F;
//  float pressure_fake = -56.78F;
//  memcpy(&vn_binary1_buffer_[index], &temp_fake, 4); index += 4; // Fake Temp
//  memcpy(&vn_binary1_buffer_[index], &pressure_fake, 4); index += 4; // Fake Pressure
  memcpy(&vn_binary1_buffer_[index], &gps_status_, 2); index += 2;
  uint16_t checksum = CalculateVNChecksum(&vn_binary1_buffer_[1], kVnBinary1Length-3);
  vn_binary1_buffer_[index] = checksum >> 8; index++;
  vn_binary1_buffer_[index] = checksum & 0xFF;
  int first_write_length = random(kVnBinary1Length);
    // Stress test the parser
//    mDebugPort.write(vn_binary1_buffer_, first_write_length);
//    delay(20);
//  mDebugPort.write(&vn_binary1_buffer_[first_write_length], kVnBinary1Length-first_write_length);
 
  ins_port_.write(vn_binary1_buffer_, kVnBinary1Length);
}

uint16_t CalculateVNChecksum(byte * data, unsigned int len)
{
  unsigned int i = 0;
  uint16_t crc = 0;
  for (i = 0; i < len; i++) {
    crc = (byte)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (byte)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}
