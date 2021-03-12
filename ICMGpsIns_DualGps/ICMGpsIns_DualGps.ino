#include <Arduino.h>
#include "wiring_private.h"
#include <Icm20948.h>
#include <SensorTypes.h>
#include "Icm20948MPUFifoControl.h"
#include <RazorSerial.h>
#include <RazorICM.h>
#include <RazorConstants.h>
#include <SevenStateEKF.h>
#include <UBX_Parser.h>
#include <Wire.h>

#define debug_port_ SerialUSB
#define ins_port_ Serial2

#define DEBUG
#ifdef DEBUG
#define DebugPrint(x) debug_port_.print(x)
#define DebugPrintDec(x) debug_port_.print(x, DEC)
#define DebugPrintHex(x) debug_port_.print(x, HEX)
#define DebugPrintln(x) debug_port_.println(x)
#else
#define DebugPrint(x)
#define DebugPrintDec(x)
#define DebugPrintHex(x)
#define DebugPrintln(x)
#endif

long led_time_prev_ms_ = 0;
long loop_time_prev_ms_ = 0;
const long kLedTimeThresholdMs = 250;
const long kLoopTimeThresholdMs = 10000;
bool led_on_ = false;
int gyro_count_ = 0;

// Serial Monitor port


#define INS_TX_PIN 6
#define INS_RX_PIN 7
//// ---- Begin SERCOM Definitions (Additional serial port)
Uart ins_port_ (&sercom5, INS_RX_PIN, INS_TX_PIN, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM5_Handler()
{
  ins_port_.IrqHandler();
}

// GPS Definitions
HardwareSerial & mGpsPort = Serial1;
const String USING_GPS_PORT = "Serial1";
UBX_Parser gps_; // This parses received characters
//gps_fix fix_;
const long fGpsBaud = 115200;

double lat0_;// = 42.315788 * DEG2RAD;
double lon0_;// = -122.961493 * DEG2RAD;
const double kEarthRadius = 6371008.8; // [m]
double gps_latitude_;
double gps_longitude_;
float gps_position_[3];
float gps_velocity_[3];
float gps_speed_kn_;
float gps_course_;
byte gps_status_ = 0;
bool gps_home_established_ = false;
long system_time_prev_us_;
// IMU Definitions
#define RESET_PIN A0
#define INTERRUPT_PIN A1

// ICM Definitions

int rc = 0;
#define THREE_AXES 3
static int unscaled_bias[THREE_AXES * 2];
/* FSR configurations */
int32_t cfg_acc_fsr = 4; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
static const uint8_t dmp3_image[] =
{
#include "icm20948_img.dmp3a.h"
};
/*
  Mounting matrix configuration applied for Accel, Gyro and Mag
*/

static const float cfg_mounting_matrix[9] = {
  1.f, 0, 0,
  0, 1.f, 0,
  0, 0, 1.f
};


static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
  INV_SENSOR_TYPE_ACCELEROMETER,
  INV_SENSOR_TYPE_GYROSCOPE,
  INV_SENSOR_TYPE_RAW_ACCELEROMETER,
  INV_SENSOR_TYPE_RAW_GYROSCOPE,
  INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
  INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
  INV_SENSOR_TYPE_BAC,
  INV_SENSOR_TYPE_STEP_DETECTOR,
  INV_SENSOR_TYPE_STEP_COUNTER,
  INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
  INV_SENSOR_TYPE_ROTATION_VECTOR,
  INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
  INV_SENSOR_TYPE_MAGNETOMETER,
  INV_SENSOR_TYPE_SMD,
  INV_SENSOR_TYPE_PICK_UP_GESTURE,
  INV_SENSOR_TYPE_TILT_DETECTOR,
  INV_SENSOR_TYPE_GRAVITY,
  INV_SENSOR_TYPE_LINEAR_ACCELERATION,
  INV_SENSOR_TYPE_ORIENTATION,
  INV_SENSOR_TYPE_B2S
};
#define AK0991x_DEFAULT_I2C_ADDR  0x0C  /* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E  /* The secondary I2C address for AK0991x Magnetometers */

#define ICM_I2C_ADDR_REVA      0x68  /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB     0x69  /* I2C slave address for INV device on Rev B board */

#define AD0_VAL   1     // The value of the last bit of the I2C address.
uint8_t I2C_Address = 0x69;
inv_icm20948_t icm_device_;
bool new_icm_data_ = false;

RazorICM imu_;
float accel_body_[3];
float gyro_float_[3];
float quat_[4];

// Main Loop Defitions
unsigned int main_loop_counter_ = 0;
const unsigned int kMainLoopCounterMax = 100; // IMU frequency

//const float kImuSampleTime = 1.0 / ((float)kMainLoopCounterMax);

// Frequencies
const unsigned int kPublishInsFrequency = 50;
const unsigned int kPublishImuFrequency = 1;
// Intervals (calculated from frequencies)
const unsigned int kPublishInsInterval = kMainLoopCounterMax / kPublishInsFrequency;
const unsigned int kPublishImuInterval = kMainLoopCounterMax / kPublishImuFrequency;
// Values (so we can offset things happening at the same frequency)
const unsigned int kPublishImuValue = 0;
const unsigned int kPublishInsValue = 0;


double imu_time_;
double imu_time_prev_ = 0;
float attitude_[4]; // roll,pitch,yaw,magYaw
long imu_time_since_boot_;
bool attitude_established_ = false;

// Heading
double rel_pos_time_;
double rel_pos_time_prev_ = 0;
float rel_pos_yaw_ = 0.F;
float antenna_offset_ = PI_2;
bool heading_established_ = false;

// INS Definitions
float ekf_position_[3];
float ekf_velocity_[3];
float mSpeedOverGround;
float mCourseOverGround;
SevenStateEKF ekf_;
double gps_fix_time_;
double gps_fix_time_prev_ = 0;
bool gps_valid_fix_ = false;
const int kVnBinary1Length = 88;//108;
byte vn_binary1_buffer_[kVnBinary1Length];
// MISC definitions
boolean bUseGps = true;
unsigned long imuStartTime;
unsigned long gpsStartTime;
unsigned long totalStartTime;
boolean bPrintAnything = false;
boolean bEcho = false;

void setup()
{
  system_time_prev_us_ = millis();
  //------------- Debug Port Setup BEGIN -----------------
#ifdef DEBUG
  debug_port_.begin(115200);
  while (!debug_port_) {}
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  //------------- Debug Port Setup END -----------------

  //------------- IMU Setup BEGIN--------------
  DebugPrintln("ICM Setup...");
  IcmSetup();

  DebugPrintln("Done!");
  //------------- IMU Setup END--------------
  ins_port_.begin(115200);
  
  pinPeripheral(INS_TX_PIN, PIO_SERCOM);
  pinPeripheral(INS_RX_PIN, PIO_SERCOM);
  // ----------- Clock Setup BEGIN --------------
  //  publish_pimu_time_prev_ms_ = millis();
  //  publish_pins1_time_prev_ms_ = publish_pimu_time_prev_ms_+1;
  // ----------- Clock Setup END --------------

  // ----------- Publish Setup BEGIN --------------
  //VN Binary1
  vn_binary1_buffer_[0] = 0xFA;
  vn_binary1_buffer_[1] = 1;
  //------------- GPS Setup BEGIN --------------
  if (bUseGps) {
    DebugPrint(F("Gps Setup..."));
    // Start the UART for the GPS device
    mGpsPort.begin(fGpsBaud);
    mGpsPort.setTimeout(1);
    CycleGps();
    DebugPrintln(F("Done!"));
  }
  //------------- GPS Setup END --------------
}


void loop()
{
  ClockTasks();
  PollIcm();
  if (imuTasks()) {
    //    printAttitude();
    //printGyroFloat();
    if ((main_loop_counter_ % kPublishInsInterval) == kPublishInsValue) {
      ekf_.GetPosition(ekf_position_);
      ekf_.GetVelocity(ekf_velocity_);
//      PublishVnBinary1();
      //            printAttitude();
    }

    main_loop_counter_++;
    if (main_loop_counter_ >= kMainLoopCounterMax) {
      main_loop_counter_ = 0;
      //      if (bPrintAnything) {
      //        debug_port_.println();
      //    debug_port_.println(millis() / 1000.0f, 4);
      //        //        DebugPrintln(String(millis()));
      //        printAttitude();
      //        //        printAccelBody();
      //        //        printGps();
      //        //                printPosition();
      //        //        printVelocity();
      //      }
    }
  }
  gpsTasks();

}
