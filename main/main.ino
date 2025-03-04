#include <CytronMotorDriver.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define _PI 3.14159265358979323846

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;

SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;
QMC5883LCompass compass;

CytronMD Fmotor(PWM_DIR, 5, 4);
CytronMD Tmotor(PWM_DIR, 6, 7);

float targetLat = 13.7563;
float targetLng = 100.5018;

// section speed control------------------------------------

int s_speed = 50, r_speed = 50, l_speed = 50, b_speed = -50;  
int TR_speed = -150, TL_speed = 150;
int SW = 150;
int L45 = 2000, L90 = 4000;
int R45 = 2000, R90 = 4000; 

//----------------------------------------------------------

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  
  compass.init();
  compasscalibration();
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  if (gps.location.isValid()) {
    float currentLat = gps.location.lat();
    float currentLng = gps.location.lng();

    float distance = getDistance(currentLat, currentLng, targetLat, targetLng);

    if (distance < 2) { //meters tolerance
      stop();
      Serial.println("Arrived at destination!");
      return;
    }

    float targetBearing = getBearing(currentLat, currentLng, targetLat, targetLng);
    float currentHeading = getHeading();

    // Adjust movement direction
    float turnAngle = targetBearing - currentHeading;

    if (turnAngle > 180) turnAngle -= 360;
    if (turnAngle < -180) turnAngle += 360;

    if (turnAngle > 10) {
      right();
    } else if (turnAngle < -10) {
      left();
    } else {
      forward();
    }

    Serial.print("Current Lat: "); Serial.print(currentLat);
    Serial.print(" | Current Lng: "); Serial.println(currentLng);
    Serial.print("Distance to Target: "); Serial.print(distance); Serial.println(" meters");
    Serial.print("Current Heading: "); Serial.print(currentHeading);
    Serial.print(" | Target Bearing: "); Serial.println(targetBearing);
  }
}

float getDistance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);

  float a = sin(dLat / 2) * sin(dLat / 2) + 
            cos(radians(lat1)) * cos(radians(lat2)) * 
            sin(dLon / 2) * sin(dLon / 2);
    
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c; // Distance in meters
}

float getBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);

  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) - 
            sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
    
  float bearing = atan2(y, x) * 180.0 / _PI;
  if (bearing < 0) bearing += 360;
  return bearing;
}

float getHeading() {
  compass.read();
  int x = compass.getX();
  int y = compass.getY();

  float heading = atan2(y, x) * 180.0 / _PI;
  if (heading < 0) heading += 360;

  heading += 3.2;
  if (heading > 360) heading -= 360;
  
  return heading;
}

void compasscalibration() {
  Serial.println("Move the sensor in all directions for calibration...");

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    compass.calibrate();
  }

  Serial.println("Calibration complete. Apply these settings in your code:");
  Serial.print("compass.setCalibrationOffsets(");
  Serial.print(compass.getCalibrationOffset(0)); Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(1)); Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(2)); Serial.println(");");

  Serial.print("compass.setCalibrationScales(");
  Serial.print(compass.getCalibrationScale(0)); Serial.print(", ");
  Serial.print(compass.getCalibrationScale(1)); Serial.print(", ");
  Serial.print(compass.getCalibrationScale(2)); Serial.println(");");
}

void stop() {
  Fmotor.setSpeed(0);
  Tmotor.setSpeed(0);
}

void forward() {
  Fmotor.setSpeed(s_speed);
}

void right() {
  Tmotor.setSpeed(TR_speed);
  delay(100);
  Fmotor.setSpeed(r_speed);
}

void left() {
  Tmotor.setSpeed(TL_speed);
  delay(100);
  Fmotor.setSpeed(l_speed);
}

void backward() {
  Fmotor.setSpeed(b_speed);
}

// Adjust Wheel Alignment
void wsl() { Tmotor.setSpeed(-SW); delay(200); }
void wsr() { Tmotor.setSpeed(SW); delay(200); }

// U-turns
void utl() { left(); delay(9000); wsl(); stop(); delay(1000); }
void utr() { right(); delay(8000); wsr(); stop(); delay(1000); }

// Reverse Turns
void b_right() { Tmotor.setSpeed(TR_speed); delay(100); Fmotor.setSpeed(-r_speed); }
void b_left() { Tmotor.setSpeed(TL_speed); delay(100); Fmotor.setSpeed(-l_speed); }
