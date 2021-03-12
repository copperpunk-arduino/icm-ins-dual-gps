void CycleGps()
{
  while (gps_.Read(&mGpsPort));
}

void gpsTasks()
{
  while (gps_.Read(&mGpsPort));
  if (gps_.IsNewFix()) {
    gps_.ClearFix();
    //    fix_ = ublox_gps_.read();
    gps_status_ = gps_.FixType();
    //    debugPrintln("cs: " + String(gps_fix_time_cs));
    //      if (fix_.valid.time && fix_.valid.date) {
    gps_fix_time_ =  gps_.TimeFix();// uint64_t(fix_.dateTime * 100 + fix_.dateTime_cs);
    //    debugPrintln("time: " + String(gps_fix_time_, 6));
    //    debugPrintln("time_prev: " + String(gps_fix_time_prev_, 6));
    system_time_prev_us_ = micros();
    float dt = (gps_fix_time_ - gps_fix_time_prev_);
    //    debugPrintln("gps dt: " + String(dt, 4));
    if (dt > 0) {
      //      debugPrintln("dt: " + String(dt, 4));
      gps_fix_time_prev_ = gps_fix_time_;

      if (gps_status_ > 2) {
        gps_valid_fix_ = true;
        gps_latitude_ = gps_.Latitude() * (1e-7) * DEG2RAD;
        gps_longitude_ = gps_.Longitude() * (1e-7) * DEG2RAD;
        gps_position_[2] = gps_.Height() * (1e-3);
        if (!gps_home_established_) {
          DebugPrintln("reset origin");
          printGps();
          resetOrigin();
          gps_home_established_ = true;
          system_time_prev_us_ = micros();
        }

        calculateDeltaXDeltaYFromLatLon(gps_position_, gps_latitude_, gps_longitude_);
        //          gps_speed_kn_ =  fix_.speed() * 0.2778;
        //          gps_course_ = fix_.heading() * DEG2RAD;
        //          fix_.calculateNorthAndEastVelocityFromSpeedAndHeading();
        gps_velocity_[0] = gps_.VNorth() / 1000.0F; //gps_speed_ * cos(gps_course_);
        gps_velocity_[1] = gps_.VEast() / 1000.0F; //gps_speed_ * sin(gps_course_);
        gps_velocity_[2] = -gps_.VDown() / 1000.0F;
        //        debugPrintln("UFP: " + String(gps_position_[0]) + "/" + String(gps_position_[1]));
        //        DebugPrint(F("ufg."));
        ekf_.UpdateFromGps(gps_position_, gps_velocity_);
        //        DebugPrintln(F("$"));
      }
    }
    gps_valid_fix_ = false;

  }
  if (gps_.IsNewRelPos()) {
    gps_.ClearRelPos();
    rel_pos_time_ =  gps_.TimeFixRelPos();
    //    debugPrintln("rel pos fix: " + String(rel_pos_time_,5));
    float dt = (rel_pos_time_ - rel_pos_time_prev_);
    //    debugPrintln("rel_pos dt: " + String(dt, 4));
    if (dt > 0) {
      rel_pos_time_prev_ = rel_pos_time_;
      rel_pos_yaw_ = gps_.RelHeading() - antenna_offset_;
      //      debugPrintln("rel yaw: " + String(rel_pos_yaw_ * RAD2DEG, 2));
      if (rel_pos_yaw_ < 0) {
        rel_pos_yaw_ += TWO_PI;
      }
      if (rel_pos_yaw_ > TWO_PI) {
        rel_pos_yaw_ -= TWO_PI;
      }
      float delta_yaw;
      if (!heading_established_) {
        delta_yaw = ekf_.SetHeading(rel_pos_yaw_);
        heading_established_ = true;
      } else {
        //              imu_.calculateRollPitchYaw();
        //                debugPrintln("rel yaw inside: " + String(rel_pos_yaw_ * RAD2DEG, 2));
        //        DebugPrint(F("ufh:"));
        delta_yaw = ekf_.UpdateFromHeading(rel_pos_yaw_);
        //        DebugPrintln(F("$"));
        DebugPrintln("delta yaw: " + String(delta_yaw * RAD2DEG, 2));
      }
      imu_.RotateDeltaYaw(delta_yaw);
    }
  }
}

void calculateDeltaXDeltaYFromLatLon(float deltaXdeltaY[], double lat1, double lon1)
{
  double dpsi = log(tan(PI_4 + lat1 / 2.0) / tan(PI_4 + lat0_ / 2.0));
  double dLat = lat1 - lat0_;
  double dLon = lon1 - lon0_;
  double q;
  if (abs(dpsi) > 0.000001) {
    q = dLat / dpsi;
  } else {
    q = cos(lat0_);
  }
  deltaXdeltaY[0] = dLat * EARTH_RADIUS_M;
  deltaXdeltaY[1] = q * dLon * EARTH_RADIUS_M;
}

void calculateLatLonFromDeltaXDeltaY(float deltaXdeltaY[], double & lat1, double & lon1)
{
  double dLat = deltaXdeltaY[0] / EARTH_RADIUS_M;
  lat1 = lat0_ + dLat;
  double dpsi = log(tan(PI_4 + lat1 / 2.0) / tan(PI_4 + lat0_ / 2.0));
  double q;
  if (abs(dpsi) > 0.000001) {
    q = dLat / dpsi;
  } else {
    q = cos(lat0_);
  }
  double dLon = (deltaXdeltaY[1] / EARTH_RADIUS_M) / q;
  lon1 = lon0_ + dLon;
}

void resetOrigin()
{
  lat0_ = gps_latitude_;
  lon0_ = gps_longitude_;
  ekf_.SetAltitude(gps_position_[2]);
  //calculateDeltaXDeltaYFromLatLong(aGpsPosition, lat0, long0, gps_latitude_, gps_longitude_);
  printOrigin();
  //  mRazorIns.resetXKalman(aGpsPosition[0],aGpsVelocityXY[0]);
  //  mRazorIns.resetYKalman(aGpsPosition[1],aGpsVelocityXY[1]);
  //  mRazorIns.resetZKalman(mMSBaroAltitude);
}
