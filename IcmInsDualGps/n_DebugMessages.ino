void printAttitude()
{
  debugPrintln("RPY[deg]: " + String(attitude_rad_[0] * kRad2Deg) + ", " + String(attitude_rad_[1] * kRad2Deg) + ", " + String(attitude_rad_[2] * kRad2Deg));
}

void printTime()
{
  debugPrintln(String(micros() / 1000000.0, 3));
}

void printAccelBody()
{
  debugPrintln("Accel[mpss]: " + String(accel_body_mpss_[0], 3) + ", " + String(accel_body_mpss_[1], 3) + ", " + String(accel_body_mpss_[2], 3));
}

void printGyroFloat()
{
  debugPrintln("Gyro[dps]: " + String(gyro_rps_[0] * kRad2Deg) + ", " + String(gyro_rps_[1] * kRad2Deg) + ", " + String(gyro_rps_[2] * kRad2Deg));
}

void printQuat()
{
  debugPrintln("quat: " + String(quat_[0], 4) + ", " + String(quat_[1], 4) + ", " + String(quat_[2], 4) + ", " + String(quat_[3], 4));
}
//

void printGps()
{
  debugPrintln("status: " + String(gps_status_));
  debugPrintln("lat/lon/alt: " + String(gps_latitude_rad_ * kRad2Deg, 7) + "/" + String(gps_longitude_rad_ * kRad2Deg, 7) + "/" + String(gps_position_rrm_[2], 2));
  debugPrintln("GPS Vel:" + String(gps_velocity_mps_[0], 3) + "/" + String(gps_velocity_mps_[1], 3) + "/" + String(gps_velocity_mps_[2], 3));
}

void printPosition()
{
  debugPrintln("pos X/Y/Z: " + String(ekf_position_[0], 2) + "/" + String(ekf_position_[1], 2) + "/" + String(ekf_position_[2], 2));
}

void printVelocity()
{
  debugPrintln("vel X/Y/Z: " + String(ekf_velocity_[0], 2) + "/" + String(ekf_velocity_[1], 2) + "/" + String(ekf_velocity_[2], 2));
}

void printOrigin()
{
  debugPrintln("Lat/Lon origin: " + String(origin_latitude_rad_ * kRad2Deg, 7) + "/" + String(origin_longitude_rad_ * kRad2Deg, 7));
}
