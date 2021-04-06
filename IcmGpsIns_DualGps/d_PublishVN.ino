#define field_bit(x) (1 << (x))

void publishVnBinary1()
{
  unsigned long t = micros();
  debugPrintln("publish: " + String(t * (1e-3)));

  // Fields 1
  // 1: TimeGPS (8)
  // 3: YPR (12)
  // 5: Body rate (12)
  // 6: Position-LLA (24)
  // 7: Velocity-NED (12)
  //-----------------------
  // Fields 2
  // 12: INS status (2)
  uint16_t fields_word = field_bit(1) | field_bit(3) | field_bit(5) | field_bit(6) | field_bit(7) | field_bit(12);
  byte fields_1 = fields_word & 0xFF;
  byte fields_2 = fields_word >> 8;
  vn_binary1_buffer_[2] = fields_1;
  vn_binary1_buffer_[3] = fields_2;
  // Sum of Data: 8 + 12 + 12 + 24 + 12 + 2 = 70
  // Message Length: 70 + 6 = 76
  int index = 4;
  // GPS time
  long current_time_us = micros();
  long time_since_last_gps_update_us = current_time_us - system_time_prev_us_;
  uint64_t gps_time_ns = gps_fix_time_sec_ * (1e9) + (time_since_last_gps_update_us)*1000;
  // Attitude
  float yaw_deg = attitude_rad_[2] * kRad2Deg;
  float pitch_deg = attitude_rad_[1] * kRad2Deg;
  float roll_deg = attitude_rad_[0] * kRad2Deg;
  memcpy(&vn_binary1_buffer_[index], &gps_time_ns, sizeof(gps_time_ns));
  index += sizeof(gps_time_ns);
  memcpy(&vn_binary1_buffer_[index], &yaw_deg, sizeof(yaw_deg));
  index += sizeof(yaw_deg);
  memcpy(&vn_binary1_buffer_[index], &pitch_deg, sizeof(pitch_deg));
  index += sizeof(pitch_deg);
  memcpy(&vn_binary1_buffer_[index], &roll_deg, sizeof(roll_deg));
  index += sizeof(roll_deg);
  // Gyro
  memcpy(&vn_binary1_buffer_[index], &gyro_rps_[0], sizeof(gyro_rps_));
  index += sizeof(gyro_rps_);
  // Position
  double lat_deg;
  double lon_deg;
  calculateLatLonFromDeltaXDeltaY(ekf_position_, lat_deg, lon_deg);
  lat_deg *= kRad2Deg;
  lon_deg *= kRad2Deg;
  double height_double = (double)ekf_position_[2];
  memcpy(&vn_binary1_buffer_[index], &lat_deg, sizeof(lat_deg));
  index += sizeof(lat_deg);
  memcpy(&vn_binary1_buffer_[index], &lon_deg, sizeof(lon_deg));
  index += sizeof(lon_deg);
  memcpy(&vn_binary1_buffer_[index], &height_double, sizeof(height_double));
  index += sizeof(height_double);
  // Velocity
  memcpy(&vn_binary1_buffer_[index], &ekf_velocity_[0], sizeof(ekf_velocity_));
  index += sizeof(ekf_velocity_);
  // INS Status
  memcpy(&vn_binary1_buffer_[index], &gps_status_, sizeof(gps_status_));
  index += sizeof(gps_status_);
  // Checksum
  uint16_t checksum = calculateVnChecksum(&vn_binary1_buffer_[1], kVnBinary1Length - 3);
  vn_binary1_buffer_[index] = checksum >> 8;
  vn_binary1_buffer_[index+1] = checksum & 0xFF;
  // Publish
  ins_port.write(vn_binary1_buffer_, kVnBinary1Length);
}

uint16_t calculateVnChecksum(byte *data, unsigned int len)
{
  unsigned int i = 0;
  uint16_t crc = 0;
  for (i = 0; i < len; i++)
  {
    crc = (byte)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (byte)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}