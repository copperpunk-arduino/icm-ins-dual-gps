boolean imuTasks()
{
  if (new_icm_data_)
  {
    new_icm_data_ = false;
    imu_count_++;
    imu_time_us_ = micros();
    float dt = (imu_time_us_ - imu_time_prev_us_) * (1e-6);
    //  DebugPrintln("dt: " + String(dt, 4));
    DebugPrintln("imu: " + String(imu_time_us_*(1e-6), 4));
    imu_time_prev_us_ = imu_time_us_;

    if (!attitude_established_)
    {
      //        debug_port_.print("soft reset");
      //      imu_.getAccelBody(accel_body_);
      //      imu_.softReset(accel_body_);
      dt = 0.0f;
      attitude_established_ = true;
    }

    imu_.updateImuWithQuat(quat_);
    imu_.getAttitudeRad(attitude_rad_);
    //      // Update EKF
    if (!gps_valid_fix_)
    {
      dt = 0.0f;
    }
    // else
    // {
    //   DebugPrintln("valid");
    // }
    ekf_.Predict(attitude_rad_, accel_body_mpss_, dt);
    return true;
  }
  //  }
  return false;
  //  */
}
