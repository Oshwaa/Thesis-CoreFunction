#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
SoftwareSerial SerialGPS(4, 5);

float Latitude, Longitude;
int year, month, date, hour, minute, second;
String DateString, TimeString, LatitudeString, LongitudeString;

void setup() {
  Serial.begin(115200); // Set baud rate for Serial monitor
  SerialGPS.begin(9600); // Set baud rate for GPS module
  Serial.println("Starting GPS data capture...");
}

void loop() {
  // Update GPS data continuously
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        Latitude = gps.location.lat();
        Longitude = gps.location.lng();
      }

      if (gps.date.isValid()) {
        date = gps.date.day();
        month = gps.date.month();
        year = gps.date.year();
      }
    }
  }

  // Check if '1' is received from the serial monitor
  if (Serial.available() > 0) {     //CHANGE LOGIC if rfid tapped get locations blablabla
    char inputChar = Serial.read();
    if (inputChar == '1') {
      // If '1' is received, print the GPS data
      Serial.println("----- GPS Data -----");
      Serial.print("Latitude: ");
      Serial.println(Latitude, 6); // Print latitude with 6 decimal places
      Serial.print("Longitude: ");
      Serial.println(Longitude, 6); // Print longitude with 6 decimal places
      Serial.print("Date: ");
      Serial.print(year);
      Serial.print("/");
      if (month < 10) Serial.print("0");
      Serial.print(month);
      Serial.print("/");
      if (date < 10) Serial.print("0");
      Serial.println(date);
      Serial.println();
    }
  }
}
