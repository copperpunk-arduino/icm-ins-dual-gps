void cycleGps()
{
  while (gps_.checkForMessage(&gps_port))
  {
  }
}

void gpsTasks()
{
  // detachPublishVnInterrupt();
  // long t = micros();
  while (gps_.checkForMessage(&gps_port))
  {
    DebugPrintln("new gps message: " + String(gps_.msgId()));
  }
  if (gps_.isNewFix())
  {
    gps_.clearFix();
    //    fix_ = ublox_gps_.read();
    gps_status_ = gps_.fixType();
    //    debugPrintln("cs: " + String(gps_fix_time_cs));
    //      if (fix_.valid.time && fix_.valid.date) {
    gps_fix_time_sec_ = gps_.timeFixSec(); // uint64_t(fix_.dateTime * 100 + fix_.dateTime_cs);
    DebugPrintln("gps time: " + String(gps_fix_time_sec_, 6));
    //    debugPrintln("time_prev: " + String(gps_fix_time_prev_, 6));
    system_time_prev_us_ = micros();
    float dt = (gps_fix_time_sec_ - gps_fix_time_prev_sec_);
    DebugPrintln("gps dt: " + String(dt, 4));
    if (dt > 0)
    {
      //      debugPrintln("dt: " + String(dt, 4));
      gps_fix_time_prev_sec_ = gps_fix_time_sec_;

      if (gps_status_ > 2)
      {
        gps_valid_fix_ = true;
        gps_latitude_rad_ = gps_.latitudeDeg() * (1e-7) * kDeg2Rad;
        gps_longitude_rad_ = gps_.longitudeDeg() * (1e-7) * kDeg2Rad;
        gps_position_rrm_[2] = gps_.heightMm() * (1e-3);
        if (!gps_home_established_)
        {
          DebugPrintln("reset origin");
          printGps();
          resetOrigin();
          gps_home_established_ = true;
          system_time_prev_us_ = micros();
        }

        calculateDeltaXDeltaYFromLatLon(gps_position_rrm_, gps_latitude_rad_, gps_longitude_rad_);
        gps_velocity_mps_[0] = gps_.vNorthMmps() * (1e-3);
        gps_velocity_mps_[1] = gps_.vEastMmps() * (1e-3);
        gps_velocity_mps_[2] = -gps_.vDownMmps() * (1e-3);
        DebugPrintln("UFP: " + String(gps_position_rrm_[0]) + "/" + String(gps_position_rrm_[1]));
        //        DebugPrint(F("ufg."));
        ekf_.UpdateFromGps(gps_position_rrm_, gps_velocity_mps_);
        ekf_.GetPosition(ekf_position_);
        ekf_.GetVelocity(ekf_velocity_);

        //        DebugPrintln(F("$"));
      }
      else
      {
        gps_valid_fix_ = false;
      }
    }
  }
  if (gps_.isNewRelHdg())
  {
    gps_.clearRelHdg();
    rel_heading_time_sec_ = gps_.timeFixRelHdgSec();
    //    debugPrintln("rel pos fix: " + String(rel_heading_time_sec_,5));
    float dt = (rel_heading_time_sec_ - rel_heading_time_prev_sec_);
    //    debugPrintln("rel_pos dt: " + String(dt, 4));
    DebugPrintln("rel dist: " + String(gps_.relPositionDistanceMm(), 1));
    if ((dt > 0) && gps_.isRelPositionDistanceWithinThreshold(kExpectedAntennaDistanceMm, kAntenntaDistanceAccuracyThresholdMm))
    {
      rel_heading_time_prev_sec_ = rel_heading_time_sec_;
      rel_heading_rad_ = (gps_.relHeadingDeg() - antenna_offset_deg_) * kDeg2Rad;
      DebugPrintln("rel yaw: " + String(rel_heading_rad_ * kRad2Deg, 2));
      if (rel_heading_rad_ < 0)
      {
        rel_heading_rad_ += kTwoPi;
      }
      if (rel_heading_rad_ > kTwoPi)
      {
        rel_heading_rad_ -= kTwoPi;
      }
      float delta_yaw_rad;
      if (!heading_established_)
      {
        delta_yaw_rad = ekf_.SetHeading(rel_heading_rad_);
        heading_established_ = true;
      }
      else
      {
        //              imu_.calculateRollPitchYaw();
        //                debugPrintln("rel yaw inside: " + String(rel_heading_rad_ * kRad2Deg, 2));
        //        DebugPrint(F("ufh:"));
        delta_yaw_rad = ekf_.UpdateFromHeading(rel_heading_rad_);
        //        DebugPrintln(F("$"));
        // DebugPrintln("delta yaw: " + String(delta_yaw_rad * kRad2Deg, 2));
      }
      imu_.rotateDeltaYawRad(delta_yaw_rad);
    }
  }
  // unsigned long t = micros();
  // long dt_gps = t - check_gps_time_prev_us_;
  // check_gps_time_prev_us_ = t;
  // publish_ins_message_time_remaining_ -= dt_gps;
  // if (publish_ins_message_time_remaining_ < 1)
  // {
  //   // publish_ins_message_time_remaining_ = 1;
  //   publishVnBinary1();
  // }
  // else
  // {

  //   startPublishVnInterrupt(publish_ins_message_time_remaining_);
  // }
  // = constrain(publish_ins_message_time_remaining_ - dt_gps, 1, kPublishInsIntervalUs);
  // DebugPrintln("gps: " + String(dt_gps));
}

void calculateDeltaXDeltaYFromLatLon(float deltaXdeltaY[], double lat1, double lon1)
{
  double dpsi = log(tan(kPi_4 + lat1 / 2.0) / tan(kPi_4 + lat0_ / 2.0));
  double dLat = lat1 - lat0_;
  double dLon = lon1 - lon0_;
  double q;
  if (abs(dpsi) > 0.000001)
  {
    q = dLat / dpsi;
  }
  else
  {
    q = cos(lat0_);
  }
  deltaXdeltaY[0] = dLat * kEarthRadiusM;
  deltaXdeltaY[1] = q * dLon * kEarthRadiusM;
}

void calculateLatLonFromDeltaXDeltaY(float deltaXdeltaY[], double &lat1, double &lon1)
{
  double dLat = deltaXdeltaY[0] / kEarthRadiusM;
  lat1 = lat0_ + dLat;
  double dpsi = log(tan(kPi_4 + lat1 / 2.0) / tan(kPi_4 + lat0_ / 2.0));
  double q;
  if (abs(dpsi) > 0.000001)
  {
    q = dLat / dpsi;
  }
  else
  {
    q = cos(lat0_);
  }
  double dLon = (deltaXdeltaY[1] / kEarthRadiusM) / q;
  lon1 = lon0_ + dLon;
}

void resetOrigin()
{
  lat0_ = gps_latitude_rad_;
  lon0_ = gps_longitude_rad_;
  ekf_.SetAltitude(gps_position_rrm_[2]);
  //calculateDeltaXDeltaYFromLatLong(aGpsPosition, lat0, long0, gps_latitude_rad_, gps_longitude_rad_);
  printOrigin();
  //  mRazorIns.resetXKalman(aGpsPosition[0],aGpsVelocityXY[0]);
  //  mRazorIns.resetYKalman(aGpsPosition[1],aGpsVelocityXY[1]);
  //  mRazorIns.resetZKalman(mMSBaroAltitude);
}

// void startGpsInterrupt()
// {
//   check_gps_timer_.attachInterruptInterval(kCheckGpsIntervalUs, gpsTasks);
// }
