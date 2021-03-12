boolean imuTasks() {
  if (new_icm_data_) {
    new_icm_data_ = false;
    gyro_count_++;
    imu_time_ = micros();
    float dt = (imu_time_ - imu_time_prev_) / 1000000.0f;
    //    debugPrintln("dt: " + String(dt, 4));
    imu_time_prev_ = imu_time_;



    if (!attitude_established_) {
      //        debug_port_.print("soft reset");
      //      imu_.getAccelBody(accel_body_);
      //      imu_.softReset(accel_body_);
      dt = 0.0f;
      attitude_established_ = true;
    }

    imu_.UpdateImu(quat_);
    imu_.GetAttitude(attitude_);
    printAttitude();
//    printQuat();
    //    printGyroFloat();
    //    imu_.calculateRollPitchYaw();
    //    imu_.getAccelBody(accel_body_);
    //    imu_.getGyro(gyro_float_);
    //    attitude_[0] = imu_.getRoll();
    //    attitude_[1] = imu_.getPitch();
    //    attitude_[2] = imu_.getYaw();

    //      // Update EKF
    if (!gps_valid_fix_) {
      dt = 0.0f;
    } else {
      DebugPrintln("valid");
    }
    ekf_.Predict(attitude_, accel_body_, dt);
    return true;
  }
  //  }
  return false;
  //  */
}
