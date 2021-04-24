void clockTasks()
{
  long t = micros();
#ifdef DEBUG
  // Loop Counter - Debug purpose only
  if ((t - loop_time_prev_us_) > kLoopTimeThresholdUs)
  {
    loop_time_prev_us_ = t;
    debugPrintln();
    printAttitude();
    debugPrintln("imu count:" + String(imu_count_));
    imu_count_ = 0;
  }
#endif

#ifdef USE_LED
  if ((t - led_time_prev_us_) > kLedTimeThresholdUs)
  {
    led_time_prev_us_ = t;
    led_on_ = !led_on_;
    digitalWrite(LED_BUILTIN, led_on_);
  }
#endif
}
