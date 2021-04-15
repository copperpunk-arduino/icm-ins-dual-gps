# IcmInsDualGps

Here we are trying to replicate the VectorNav [VN-300](https://www.vectornav.com/products/vn-300) with a cheaper (and less performant) substitute. This sketch will output a binary message that conforms to the VN Binary 1 message protocol, so if you ever got your hands on a VN-300, you could swap it out for this setup without any issues.<br>

We will utilize the [simpleRTK2B + heading kit](https://www.ardusimple.com/product/simplertk2b-heading-basic-starter-kit-ip67/) from ArduSimple. This kit allows you to use two GPS receivers to obtain heading information, which is a welcome replacement for a magnetometer.

Additional hardware includes an [ICM-20948 breakout board] from Sparkfun(https://www.sparkfun.com/products/15335) and the [QT Py](https://www.adafruit.com/product/4600) SAMD21 dev board from Adafruit.

In order to run this sketch you must download a few additional libraries:
*   https://github.com/ericalbers/ICM20948_DMP_Arduino
*   https://github.com/copperpunk-arduino/imu-external-yaw
*   https://github.com/copperpunk-arduino/seven-state-ekf
*   https://github.com/copperpunk-arduino/ubx-gps

Once these are installed, you can choose to publish your VectorNav binary messages over the USB serial port, or the SERCOM2 serial port, which uses the MOSI and MISO pins. This is the setup our example uses, and the code demonstrates how this is done.

For more information on how to hook up the simpleRTK2B+heading board, please refer to ArduSimple's documentation:
https://www.ardusimple.com/simplertk2heading-hookup-guide/

The final result (without antennas attached) is shown below. We've attached a [4-pin polarized connecter](http://www.hansenhobbies.com/products/connectors/pt1inlpconnectors/) for transmitting the VectorNav message over a hardware serial port, but as stated, you can use the USB serial port if you prefer.
<p align="center"><img src="https://static.wixstatic.com/media/07c139_fa161f1471624ed0a459e64061976e64~mv2.jpg" width="50%"><img src="https://static.wixstatic.com/media/07c139_f1a0a76a02804de7b966cbfdd807d9b8~mv2.jpg" width=50%</p>
<p align="center"><img src="https://static.wixstatic.com/media/07c139_6fbc901f3bc34d10bf79c02c932e6c9f~mv2.jpg" width=50%><img src="https://static.wixstatic.com/media/07c139_36ea9a80b61e4dc38220b24f4baf99d3~mv2.jpg" width="50%"></p>

# Contact Us
If you have any suggestions for improving this repository, there are a few ways to get in touch:

*   Create a new issue
*   Submit a pull request
*   Virtual meeting using our [Open Source Office Hours](https://www.copperpunk.com/service-page/open-source-office-hours)