# IcmInsDualGps

Here we are trying to replicate the VectorNav [VN-300](https://www.vectornav.com/products/vn-300) with a cheaper (and less performant) substitute. This sketch will output a binary message that conforms to the VN Binary 1 message protocol, so if you ever got your hands on a VN-300, you could swap it out for this setup without any issues.<br>

We will utilize the [simpleRTK2B + heading kit](https://www.ardusimple.com/product/simplertk2b-heading-basic-starter-kit-ip67/) from ArduSimple. This kit allows you to use two GPS receivers to obtain heading information, which is a welcome replacement for a magnetometer.

Additional hardware includes an [ICM-20948 breakout board] from Sparkfun(https://www.sparkfun.com/products/15335) and the [QT Py](https://www.adafruit.com/product/4600) SAMD21 dev board from Adafruit.

In order to run this sketch you must download a few additional libraries:
*   https://github.com/ericalbers/ICM20948_DMP_Arduino
*   https://github.com/copperpunk-arduino/imu-external-yaw
*   https://github.com/copperpunk-arduino/seven-state-ekf
*   https://github.com/copperpunk-arduino/ubx-gps

Once these are installed, you can choose to publish your VectorNav binary messages over the USB Serial port, or the SERCOM2 serial port, which uses the MOSI and MISO pins. This is the setup our example uses, and the code demonstrates how this is done.