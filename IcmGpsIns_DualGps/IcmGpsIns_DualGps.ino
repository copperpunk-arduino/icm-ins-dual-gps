#include <Arduino.h>
#include "wiring_private.h"
#include "SAMDTimerInterrupt.h"
#include <Icm20948.h>
#include <SensorTypes.h>
#include "Icm20948MPUFifoControl.h"
// #include <RazorSerial.h>
// #include "RazorICM.h"
// #include <RazorConstants.h>
#include "ImuUtils.h"
#include "UbxGps.h"
#include "SevenStateEkf.h"
#include <Wire.h>

#define debug_port Serial
#define gps_port Serial1
#define ins_port Serial2

// #define USE_LED
#define PUBLISH_VN

#define DEBUG
#ifdef DEBUG
#define DebugPrint(x) debug_port.print(x)
#define DebugPrintDec(x) debug_port.print(x, DEC)
#define DebugPrintHex(x) debug_port.print(x, HEX)
#define DebugPrintln(x) debug_port.println(x)
#else
#define DebugPrint(x)
#define DebugPrintDec(x)
#define DebugPrintHex(x)
#define DebugPrintln(x)
#endif

const float kDeg2Rad = 0.017453293f;
const float kRad2Deg = 57.295779513f;
const float kTwoPi = 6.283185307f;
const float kPi_4 = 0.785398163f;
const float kEarthRadiusM = 6371008.8f;
const float kGravity = 9.80665;

// Serial Monitor port

#define INS_TX_PIN MOSI
#define INS_RX_PIN MISO
//// ---- Begin SERCOM Definitions (Additional serial port)
Uart ins_port(&sercom2, INS_RX_PIN, INS_TX_PIN, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM2_Handler()
{
  ins_port.IrqHandler();
}
const long kInsBaud = 115200;

// GPS Definitions
UbxGps gps_; // This parses received characters
const long kGpsBaud = 115200;

double lat0_;                          // = 42.315788 * DEG2RAD;
double lon0_;                          // = -122.961493 * DEG2RAD;
const double kEarthRadius = 6371008.8; // [m]
double gps_latitude_rad_;
double gps_longitude_rad_;
float gps_position_rrm_[3]; //[rad/rad/m]
float gps_velocity_mps_[3];
float gps_speed_kn_;
float gps_course_rad_;
byte gps_status_ = 0;
bool gps_home_established_ = false;
long system_time_prev_us_;
// IMU Definitions
#define RESET_PIN A0
#define INTERRUPT_PIN A1

// ICM Definitions
static int unscaled_bias_[3 * 2]; // Three axes
/* FSR configurations */
int32_t cfg_acc_fsr = 4;    // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
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
    0, 0, 1.f};

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
    INV_SENSOR_TYPE_B2S};
#define AK0991x_DEFAULT_I2C_ADDR 0x0C   /* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR 0x0E /* The secondary I2C address for AK0991x Magnetometers */

#define ICM_I2C_ADDR_REVA 0x68 /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB 0x69 /* I2C slave address for INV device on Rev B board */

#define AD0_VAL 1 // The value of the last bit of the I2C address.
uint8_t I2C_Address = 0x69;
inv_icm20948_t icm_device_;
bool new_icm_data_ = false;

ImuUtils imu_;
float accel_body_mpss_[3];
float gyro_rps_[3];
float quat_[4];
const unsigned int kImuIntervalMs = 10;

// Clock Definitions
unsigned long led_time_prev_us_ = 0;
unsigned long loop_time_prev_us_ = 0;
const unsigned long kLedTimeThresholdUs = 250000;
const unsigned long kLoopTimeThresholdUs = 1000000;
// const unsigned long kLoop
bool led_on_ = false;
int imu_count_ = 0;

// VectorNav message publishing
const unsigned long kPublishInsIntervalUs = 20000; // 50 Hz
// const unsigned long kCheckGpsIntervalUs = 1000; // 100 Hz

unsigned long check_gps_time_prev_us_ = 0;
volatile long publish_ins_message_time_remaining_ = kPublishInsIntervalUs;
SAMDTimer publish_vn_timer_(TIMER_TCC);
// SAMDTimer check_gps_timer_(TIMER_TCC);

double imu_time_us_;
double imu_time_prev_us_ = 0;
float attitude_rad_[3]; // roll,pitch,yaw
long imu_time_since_boot_;
bool attitude_established_ = false;

// Heading
double rel_heading_time_sec_;
double rel_heading_time_prev_sec_ = 0;
float rel_heading_rad_ = 0.F;
float antenna_offset_deg_ = 90.f;
const float kExpectedAntennaDistanceMm = 1822.5f;
const float kAntenntaDistanceAccuracyThresholdMm = 200.f;
bool heading_established_ = false;

// INS Definitions
float ekf_position_[3];
float ekf_velocity_[3];
float mSpeedOverGround;
float mCourseOverGround;
SevenStateEkf ekf_;
double gps_fix_time_sec_;
double gps_fix_time_prev_sec_ = 0;
bool gps_valid_fix_ = false;
const int kVnBinary1Length = 88;
byte vn_binary1_buffer_[kVnBinary1Length];

void setup()
{
  system_time_prev_us_ = millis();
  //------------- Debug Port Setup BEGIN -----------------
#ifdef DEBUG
  debug_port.begin(115200);
  while (!debug_port)
  {
  }
#endif
#ifdef USE_LED
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  //------------- Debug Port Setup END -----------------

  //------------- IMU Setup BEGIN--------------
  DebugPrintln("ICM Setup...");
  icmSetup();
  imu_time_prev_us_ = micros();
  DebugPrintln("Done!");
  //------------- IMU Setup END--------------
  ins_port.begin(kInsBaud);

  pinPeripheral(INS_TX_PIN, PIO_SERCOM);
  pinPeripheral(INS_RX_PIN, PIO_SERCOM);
  // ----------- Clock Setup BEGIN --------------
  loop_time_prev_us_ = micros();
  led_time_prev_us_ = micros();

  // ----------- Clock Setup END --------------

  // ----------- Publish Setup BEGIN --------------
  vn_binary1_buffer_[0] = 0xFA;
  vn_binary1_buffer_[1] = 1;
#ifdef PUBLISH_VN
  // startPublishVnInterrupt(publish_ins_message_time_remaining_);
#endif
  //------------- GPS Setup BEGIN --------------
  DebugPrint(F("Gps Setup..."));
  // Start the UART for the GPS device
  gps_port.begin(kGpsBaud);
  gps_port.setTimeout(1);
  cycleGps();
  DebugPrintln(F("Done!"));
  pollIcm();
  check_gps_time_prev_us_ = micros();
  // startGpsInterrupt();
  //------------- GPS Setup END --------------
}
bool pub_ = false;
void loop()
{
  pollIcm();
  if(imuTasks()) {
    pub_ = !pub_;
  }
  if (pub_) {
    publishVnBinary1();
    pub_ = false;
  }
  gpsTasks();
  clockTasks();
}
