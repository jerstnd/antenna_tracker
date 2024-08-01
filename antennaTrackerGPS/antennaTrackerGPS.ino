#include <Servo.h>
#include <math.h>

#define PI 3.14159265358979323846
#define EARTH_RADIUS 6371000  // Earth's radius in meters

Servo panServo;
Servo tiltServo;

const int rssiPin = A1;
const int panServoPin = 6; // azimuth servo
const int tiltServoPin = 7; // elevation servo

float refLat = -7.9502284;   // Latitude in degrees
float refLon = 112.6131036;    // Longitude in degrees
float refAlt = 510;      

float targetLat, targetLon, targetAlt; // GPS Data global variable 

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud
  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);
}

void loop() {
  if (Serial.available()) {
    String nmeaSentence = Serial.readStringUntil('\n');
    //Serial.println(nmeaSentence);  // Print raw NMEA sentence for debugging

    if (nmeaSentence.startsWith("$GPGGA")) {
      parseNMEA(nmeaSentence);
    }

    float azimuth = calculateAzimuth(refLat, refLon, targetLat, targetLon);
    float elevation = calculateElevation(refLat, refLon, refAlt, targetLat, targetLon, targetAlt);

    // Map the angles to servo positions (0-180 degrees)
    int panServoPos = map(azimuth, 0, 360, 0, 180);
    int tiltServoPos = map(elevation, -90, 90, 0, 180);

    Serial.print("tilt servo: "); Serial.println(tiltServoPos, DEC);  
    Serial.print("pan servo: "); Serial.println(panServoPos, DEC);  

    // Set servo positions
    panServo.write(panServoPos);
    tiltServo.write(tiltServoPos); 
  }
}

void parseNMEA(String sentence) {
  String parts[15];
  int partIndex = 0;
  int startIndex = 0;

  // Split the sentence into parts by comma
  for (int i = 0; i < sentence.length(); i++) {
    if (sentence.charAt(i) == ',') {
      parts[partIndex] = sentence.substring(startIndex, i);
      startIndex = i + 1;
      partIndex++;
    }
  }
  parts[partIndex] = sentence.substring(startIndex);  // Last part

  // Parse latitude and longitude
  float targetLat = convertToDecimal(parts[2].toFloat(), parts[3].charAt(0));
  float targetLon = convertToDecimal(parts[4].toFloat(), parts[5].charAt(0));
  float targetAlt = parts[9].toFloat();
     
   //For Debugging
   Serial.print("Latitude: "); Serial.println(targetLat, 6);
   Serial.print("Longitude: "); Serial.println(targetLon, 6);
   Serial.print("Altitude: "); Serial.println(targetAlt, 2);  
}

float convertToDecimal(float coordinate, char hemisphere) {
  int coordinateDegrees = int(coordinate / 100);
  float coordinateMinutes = coordinate - coordinateDegrees * 100;
  float coordinateDecimal = coordinateDegrees + coordinateMinutes / 60.0;

  if (hemisphere == 'S' || hemisphere == 'W') {
    coordinateDecimal = -coordinateDecimal;
  }

  return coordinateDecimal;
}

float calculateAzimuth(float lat1, float lon1, float lat2, float lon2) {
  // Convert degrees to radians
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  float dLon = lon2 - lon1;
  float x = sin(dLon) * cos(lat2);
  float y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  float initial_bearing = atan2(x, y);
  initial_bearing = degrees(initial_bearing);
  if (initial_bearing < 0) {
    initial_bearing += 360;
  }
  return initial_bearing;
}

float calculateElevation(float lat1, float lon1, float alt1, float lat2, float lon2, float alt2) {
  float horizontalDistance = calculateDistance(lat1, lon1, lat2, lon2);
  float verticalDistance = alt2 - alt1;
  return degrees(atan2(verticalDistance, horizontalDistance));
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  float dLat = lat2 - lat1;
  float dLon = lon2 - lon1;
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = EARTH_RADIUS * c;
  return distance;
}
