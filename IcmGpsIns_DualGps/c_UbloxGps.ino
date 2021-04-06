void cycleGps()
{
  while (gps_.checkForMessage(&gps_port))
  {
  }
}

void gpsTasks()
{
  while (gps_.checkForMessage(&gps_port))
  {
    debugPrintln("new gps message: " + String(gps_.msgId()));
  }
  if (gps_.isNewFix())
  {
    gps_.clearFix();
    gps_status_ = gps_.fixType();
    gps_fix_time_sec_ = gps_.timeFixSec();
    debugPrintln("gps time: " + String(gps_fix_time_sec_, 6));
    system_time_prev_us_ = micros();
    float dt = (gps_fix_time_sec_ - gps_fix_time_prev_sec_);
    debugPrintln("gps dt: " + String(dt, 4));
    if (dt > 0)
    {
      gps_fix_time_prev_sec_ = gps_fix_time_sec_;

      if (gps_status_ > 2)
      {
        gps_valid_fix_ = true;
        gps_latitude_rad_ = gps_.latitudeDeg() * (1e-7) * kDeg2Rad;
        gps_longitude_rad_ = gps_.longitudeDeg() * (1e-7) * kDeg2Rad;
        gps_position_rrm_[2] = gps_.heightMm() * (1e-3);
        if (!gps_home_established_)
        {
          debugPrintln("reset origin");
          printGps();
          resetOrigin();
          gps_home_established_ = true;
          system_time_prev_us_ = micros();
        }

        calculateDeltaXDeltaYFromLatLon(gps_position_rrm_, gps_latitude_rad_, gps_longitude_rad_);
        gps_velocity_mps_[0] = gps_.vNorthMmps() * (1e-3);
        gps_velocity_mps_[1] = gps_.vEastMmps() * (1e-3);
        gps_velocity_mps_[2] = -gps_.vDownMmps() * (1e-3);
        debugPrintln("Lat/Lon: " + String(gps_position_rrm_[0]) + "/" + String(gps_position_rrm_[1]));
        ekf_.UpdateFromGps(gps_position_rrm_, gps_velocity_mps_);
        ekf_.GetPosition(ekf_position_);
        ekf_.GetVelocity(ekf_velocity_);
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
    float dt = (rel_heading_time_sec_ - rel_heading_time_prev_sec_);
    debugPrintln("rel dist: " + String(gps_.relPositionDistanceMm(), 1));
    if ((dt > 0) && gps_.isRelPositionDistanceWithinThreshold(kExpectedAntennaDistanceMm, kAntenntaDistanceAccuracyThresholdMm))
    {
      rel_heading_time_prev_sec_ = rel_heading_time_sec_;
      rel_heading_rad_ = (gps_.relHeadingDeg() - antenna_offset_deg_) * kDeg2Rad;
      debugPrintln("rel yaw: " + String(rel_heading_rad_ * kRad2Deg, 2));
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
        delta_yaw_rad = ekf_.UpdateFromHeading(rel_heading_rad_);
      }
      imu_.rotateDeltaYawRad(delta_yaw_rad);
    }
  }
}

void calculateDeltaXDeltaYFromLatLon(float deltaXdeltaY[], double lat1, double lon1)
{
  double dpsi = log(tan(kPi_4 + lat1 / 2.0) / tan(kPi_4 + origin_latitude_rad_ / 2.0));
  double dLat = lat1 - origin_latitude_rad_;
  double dLon = lon1 - origin_longitude_rad_;
  double q;
  if (abs(dpsi) > 0.000001)
  {
    q = dLat / dpsi;
  }
  else
  {
    q = cos(origin_latitude_rad_);
  }
  deltaXdeltaY[0] = dLat * kEarthRadiusM;
  deltaXdeltaY[1] = q * dLon * kEarthRadiusM;
}

void calculateLatLonFromDeltaXDeltaY(float deltaXdeltaY[], double &lat1, double &lon1)
{
  double dLat = deltaXdeltaY[0] / kEarthRadiusM;
  lat1 = origin_latitude_rad_ + dLat;
  double dpsi = log(tan(kPi_4 + lat1 / 2.0) / tan(kPi_4 + origin_latitude_rad_ / 2.0));
  double q;
  if (abs(dpsi) > 0.000001)
  {
    q = dLat / dpsi;
  }
  else
  {
    q = cos(origin_latitude_rad_);
  }
  double dLon = (deltaXdeltaY[1] / kEarthRadiusM) / q;
  lon1 = origin_longitude_rad_ + dLon;
}

void resetOrigin()
{
  origin_latitude_rad_ = gps_latitude_rad_;
  origin_longitude_rad_ = gps_longitude_rad_;
  ekf_.SetAltitude(gps_position_rrm_[2]);
  printOrigin();
}