#include <CytronMotorDriver.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define _PI 3.14159265358979323846
#define NUM_POINTS 6  

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;

SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;
QMC5883LCompass compass;

CytronMD Fmotor(PWM_DIR, 5, 4);
CytronMD Tmotor(PWM_DIR, 6, 7);

float targetLocation[NUM_POINTS][2] = {
    {13.72917, 100.52389}, 
    {13.59556, 100.60722}, 
    {17.53917, 102.78444}, 
    {13.85083, 100.52222}, 
    {13.36222, 100.98333}, 
    {14.97500, 102.10000}  
};

float distanceMatrix[NUM_POINTS][NUM_POINTS];

int s_speed = 50, r_speed = 50, l_speed = 50, b_speed = -50;
int TR_speed = -150, TL_speed = 150;

void setup() {
    Serial.begin(115200);
    ss.begin(GPSBaud);
    
    compass.init();
    compasscalibration();
    setupDistanceMatrix();

    int path[NUM_POINTS];
    int pathLength = findShortestPath(1, 4, path);

    Serial.print("Shortest Path: ");
    for (int i = 0; i < pathLength; i++) {
        Serial.print(path[i]);
        if (i < pathLength - 1) Serial.print(" -> ");
    }
    Serial.println();

    moveCar(path, pathLength);
}

void loop() {
}

// Move the car along the shortest path
void moveCar(int path[], int pathLength) {
    for (int i = 0; i < pathLength - 1; i++) {
        int currentPoint = path[i];
        int nextPoint = path[i + 1];

        Serial.print("Moving to waypoint ");
        Serial.println(nextPoint);
        
        moveTo(targetLocation[nextPoint][0], targetLocation[nextPoint][1]);
    }
    Serial.println("Arrived at destination! 🚗💨");
    stop();
}

// Move the car to a specific GPS coordinate
void moveTo(float targetLat, float targetLon) {
    while (true) {
        if (ss.available() > 0) gps.encode(ss.read());

        if (gps.location.isValid()) {
            float currentLat = gps.location.lat();
            float currentLon = gps.location.lng();

            float distanceToTarget = getDistance(currentLat, currentLon, targetLat, targetLon);
            if (distanceToTarget < 5) {  
                stop();
                Serial.println("Reached waypoint!");
                delay(1000);
                return;
            }

            float bearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
            adjustDirection(bearing);
            forward();
        }
    }
}

// Adjust direction using compass
void adjustDirection(float targetBearing) {
    while (true) {
        float currentHeading = getHeading();
        float diff = targetBearing - currentHeading;

        if (abs(diff) < 5) break;

        if (diff > 0) right();
        else left();
    }
    stop();
}

// Function to calculate GPS bearing
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = radians(lon2 - lon1);
    float y = sin(dLon) * cos(radians(lat2));
    float x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
    
    float bearing = atan2(y, x) * 180.0 / _PI;
    if (bearing < 0) bearing += 360;
    
    return bearing;
}

void setupDistanceMatrix() {
    for (int i = 0; i < NUM_POINTS; i++) {
        for (int j = 0; j < NUM_POINTS; j++) {
            if (i == j) {
                distanceMatrix[i][j] = 0;
            } else {
                distanceMatrix[i][j] = getDistance(targetLocation[i][0], targetLocation[i][1],
                                                   targetLocation[j][0], targetLocation[j][1]);
            }
        }
    }
}

int findShortestPath(int start, int end, int path[]) {
    float minDist[NUM_POINTS];  
    bool visited[NUM_POINTS];   
    int previous[NUM_POINTS];   

    for (int i = 0; i < NUM_POINTS; i++) {
        minDist[i] = 1e6;
        visited[i] = false;
        previous[i] = -1;
    }
    minDist[start] = 0;

    for (int i = 0; i < NUM_POINTS; i++) {
        int u = -1;
        float minValue = 1e6;

        for (int j = 0; j < NUM_POINTS; j++) {
            if (!visited[j] && minDist[j] < minValue) {
                u = j;
                minValue = minDist[j];
            }
        }

        if (u == -1) break;
        visited[u] = true;

        for (int v = 0; v < NUM_POINTS; v++) {
            if (!visited[v] && distanceMatrix[u][v] > 0) {
                float alt = minDist[u] + distanceMatrix[u][v];
                if (alt < minDist[v]) {
                    minDist[v] = alt;
                    previous[v] = u;
                }
            }
        }
    }

    int count = 0;
    for (int at = end; at != -1; at = previous[at]) {
        path[count++] = at;
    }

    for (int i = 0; i < count / 2; i++) {
        int temp = path[i];
        path[i] = path[count - 1 - i];
        path[count - 1 - i] = temp;
    }

    return count;
}

float getDistance(float lat1, float lon1, float lat2, float lon2) {
    float R = 6371000; 
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);

    float a = sin(dLat / 2) * sin(dLat / 2) + 
              cos(radians(lat1)) * cos(radians(lat2)) * 
              sin(dLon / 2) * sin(dLon / 2);

    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

float getHeading() {
    int x = compass.getX();
    int y = compass.getY();

    float heading = atan2(y, x) * 180.0 / _PI;
    if (heading < 0) heading += 360;

    heading += 3.2;
    if (heading > 360) heading -= 360;
    
    return heading;
}

void compasscalibration() {
    Serial.println("Calibrating compass...");

    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
        compass.calibrate();
    }
}

void stop() { Fmotor.setSpeed(0); Tmotor.setSpeed(0); }
void forward() { Fmotor.setSpeed(s_speed); }
void right() { Tmotor.setSpeed(TR_speed); delay(100); Fmotor.setSpeed(r_speed); }
void left() { Tmotor.setSpeed(TL_speed); delay(100); Fmotor.setSpeed(l_speed); }
void backward() { Fmotor.setSpeed(b_speed); }
