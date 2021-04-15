boolean imuTasks()
{
  if (new_icm_data_)
  {
    new_icm_data_ = false;
    imu_count_++;
    imu_time_us_ = micros();
    float dt = (imu_time_us_ - imu_time_prev_us_) * (1e-6);
    //  debugPrintln("dt: " + String(dt, 4));
    debugPrintln("imu: " + String(imu_time_us_ * (1e-6), 4));
    imu_time_prev_us_ = imu_time_us_;

    if (!attitude_established_)
    {
      dt = 0.0f;
      attitude_established_ = true;
    } else if (!gps_valid_fix_) {
      dt = 0.0f;
    }

    imu_.updateImuWithQuatDifferentialYaw(quat_);
    imu_.getAttitudeRad(attitude_rad_);

    ekf_.Predict(attitude_rad_, accel_body_mpss_, dt);
    return true;
  }
  return false;
}
