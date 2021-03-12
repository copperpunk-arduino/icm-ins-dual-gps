void PrintVector(String header, float vector[], float mult)
{
  DebugPrintln(header + String(vector[0]*mult ) + ", " + String(vector[1]*mult) + ", " + String(vector[2] * mult));
}

void printAttitude() {
//  DebugPrintln("rel yaw: " + String(rel_pos_yaw_ * RAD2DEG, 2));
  DebugPrintln("R/P/Y: " + String(attitude_[0] * RAD2DEG ) + ", " + String(attitude_[1] * RAD2DEG) + ", " + String(attitude_[2] * RAD2DEG));
}

void printTime() {
  DebugPrintln(String(micros() / 1000000.0, 3));
}

void printAccelBody() {
  DebugPrintln("A body: " + String(accel_body_[0],3) + ", " + String(accel_body_[1],3) + ", " + String(accel_body_[2],3));
}

void printGyroFloat() {
//  imu_.getGyro(gyro_float_);
  DebugPrintln("Gyro float: " + String(gyro_float_[0]* RAD2DEG) + ", " + String(gyro_float_[1]* RAD2DEG) + ", " + String(gyro_float_[2]* RAD2DEG));
}

void printQuat() {
  DebugPrintln("quat: " + String(quat_[0], 4) + ", " + String(quat_[1], 4) + ", " + String(quat_[2], 4) + ", " + String(quat_[3], 4));
}
//

void printGps() {
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
//  DebugPrintln("gps speed[kn]/course: " + String(gps_speed_kn_,2) + "/" + String(gps_course_*RAD2DEG,1));
DebugPrintln("status: " + String(gps_status_));
  DebugPrintln("lat/lon/alt: " + String(gps_latitude_*RAD2DEG,7) + "/" + String(gps_longitude_*RAD2DEG,7) + "/" + String(gps_position_[2], 2));  
//    DebugPrintln("GPS Pos:" + String(gps_position_[0], 2) + "/" + String(gps_position_[1], 2) + "/" + String(gps_position_[2], 2));
  DebugPrintln("GPS Vel:" + String(gps_velocity_[0], 3) + "/" + String(gps_velocity_[1], 3) + "/" + String(gps_velocity_[2], 3));
    
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
  DebugPrintln("pos X/Y/Z: " + String(ekf_position_[0], 2) + "/" + String(ekf_position_[1], 2) + "/"  + String(ekf_position_[2], 2));
}

void printVelocity()
{
  DebugPrintln("vel X/Y/Z: " + String(ekf_velocity_[0], 2) + "/" + String(ekf_velocity_[1], 2) + "/"  + String(ekf_velocity_[2], 2));
}

void printZ()
{
  DebugPrintln("Z pos/vel: " + String(ekf_position_[2], 2) + "/" + String(ekf_velocity_[2], 2));
}

void printOrigin()
{
  DebugPrintln("Lat/Lon origin: " + String(lat0_*RAD2DEG,7) + "/" + String(lon0_*RAD2DEG,7));
}
