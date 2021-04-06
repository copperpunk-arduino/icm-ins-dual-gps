void printAttitude()
{
  //  DebugPrintln("rel yaw: " + String(rel_pos_yaw_ * kRad2Deg, 2);
  DebugPrintln("R/P/Y: " + String(attitude_rad_[0] * kRad2Deg) + ", " + String(attitude_rad_[1] * kRad2Deg) + ", " + String(attitude_rad_[2] * kRad2Deg));
}

void printTime()
{
  DebugPrintln(String(micros() / 1000000.0, 3));
}

void printAccelBody()
{
  DebugPrintln("A body: " + String(accel_body_mpss_[0], 3) + ", " + String(accel_body_mpss_[1], 3) + ", " + String(accel_body_mpss_[2], 3));
}

void printGyroFloat()
{
  //  imu_.getGyro(gyro_rps_);
  DebugPrintln("Gyro float: " + String(gyro_rps_[0] * kRad2Deg) + ", " + String(gyro_rps_[1] * kRad2Deg) + ", " + String(gyro_rps_[2] * kRad2Deg));
}

void printQuat()
{
  DebugPrintln("quat: " + String(quat_[0], 4) + ", " + String(quat_[1], 4) + ", " + String(quat_[2], 4) + ", " + String(quat_[3], 4));
}
//

void printGps()
{
  //	DebugPrintln("gps status: " + String(gps_status_));
  //trace_all( debug_port_, mUbloxGps, mGpsFix);
  // Pos X/Y/Z
  // Vel X/Y
  // Fix Type
  // HDOP/VDOP/Sat

  //    DebugPrintln(String(mGpsFix.dateTime*100 + mGpsFix.dateTime_cs));
  //      debug_port_.println(mGpsFix.latitudeL() * (1e-7), 7);

  //      debug_port_.print("/");
  //      debug_port_.print(mGpsFix.longitudeL() * (1e-7), 7);
  //      debug_port_.print("/");
  //      DebugPrintln(String(mGpsFix.altitude(), 2));
  //    float mSpeedMps = mGpsFix.speed() * 0.2778;
  //    float mCourse = mGpsFix.heading();
  //  DebugPrintln("gps speed[kn]/course: " + String(gps_speed_kn_,2) + "/" + String(gps_course_*kRad2Deg,1));
  DebugPrintln("status: " + String(gps_status_));
  DebugPrintln("lat/lon/alt: " + String(gps_latitude_rad_ * kRad2Deg, 7) + "/" + String(gps_longitude_rad_ * kRad2Deg, 7) + "/" + String(gps_position_rrm_[2], 2));
  //    DebugPrintln("GPS Pos:" + String(gps_position_rrm_[0], 2) + "/" + String(gps_position_rrm_[1], 2) + "/" + String(gps_position_rrm_[2], 2));
  DebugPrintln("GPS Vel:" + String(gps_velocity_mps_[0], 3) + "/" + String(gps_velocity_mps_[1], 3) + "/" + String(gps_velocity_mps_[2], 3));

  //  debug_port_.print(mGpsFix.hdop);
  //  debug_port_.print("/");
  //  debug_port_.print(mGpsFix.vdop);
  //  debug_port_.print("/");
  //  DebugPrintln(mGpsFix.satellites);
}

//void printBME() {
//  debug_port_.print(mBMEAltitude);
//  debug_port_.print("/");
//  DebugPrintln(mBMETempC*1.8f + 32.0);
//}

void printPosition()
{
  DebugPrintln("pos X/Y/Z: " + String(ekf_position_[0], 2) + "/" + String(ekf_position_[1], 2) + "/" + String(ekf_position_[2], 2));
}

void printVelocity()
{
  DebugPrintln("vel X/Y/Z: " + String(ekf_velocity_[0], 2) + "/" + String(ekf_velocity_[1], 2) + "/" + String(ekf_velocity_[2], 2));
}

void printZ()
{
  DebugPrintln("Z pos/vel: " + String(ekf_position_[2], 2) + "/" + String(ekf_velocity_[2], 2));
}

void printOrigin()
{
  DebugPrintln("Lat/Lon origin: " + String(lat0_ * kRad2Deg, 7) + "/" + String(lon0_ * kRad2Deg, 7));
}
