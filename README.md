bt-intervalometer
=================

bt-intervalometer is an Arduino-based intervalometer with a bluetooth interface intended to be used in time-lapse photography.

Features:
  * Adjustable shooting interval, shutter time, and number of pictures to be taken.
  * Supports using a servo for moving the camera.
  * Bluetooth interface.

Parts:
  * arduino: Contains the Arduino firmware. Requires _TimerOne_ library (_Servo_ optional).
  * (In development) android: Android app which communitaces via bluetooth with the arduino and acts as an interface between the user and the intervalometer.
