void clockTasks()
{
  long t = micros();
// Loop Counter - Debug purpose only
#ifdef DEBUG
  if ((t - loop_time_prev_us_) > kLoopTimeThresholdUs)
  {
    loop_time_prev_us_ = t;
    DebugPrintln();
    printAttitude();
    DebugPrintln("imu count:" + String(imu_count_));
    imu_count_ = 0;
  }
#endif
// #ifdef PUBLISH_VN
//   long dt = t - publish_ins_message_time_prev_us_;
//   DebugPrintln(dt * (1e-3));

//   if ((t - publish_ins_message_time_prev_us_) >= kPublishInsMessageThresholdUs)
//   {
//     publish_ins_message_time_prev_us_ = t;
//     DebugPrintln("publish: " + String(t * (1e-3), 1));
//     // printAttitude();
//     // printPosition();
//     publishVnBinary1();
//   }
// #endif

#ifdef USE_LED
  if ((t - led_time_prev_us_) > kLedTimeThresholdUs)
  {
    led_time_prev_us_ = t;
    led_on_ = !led_on_;
    digitalWrite(LED_BUILTIN, led_on_);
  }
#endif
}
