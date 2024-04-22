#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <MFRC522.h>
#include <SPI.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <map>
#include <Servo.h>
#include <cmath>

TinyGPSPlus gps;
SoftwareSerial SerialGPS(4, 5);

double CurrLatitude, CurrLongitude;
double originlat, originlong; // Declare global variables for origin latitude and longitude

const char* ssid = "SGCRC"; // CHANGE
const char* password = "Malachi.3:10.";
const char* serverUrl = "http://10.14.1.130/phpprograms/datatest.php"; // PHP file location (local or online)
const char* balUrl = "http://10.14.1.130/phpprograms/getBal.php"; //CHANGE 
constexpr uint8_t RST_PIN = D3;10.14.1.130
constexpr uint8_t SS_PIN = D4;
constexpr uint8_t SERVO_PIN = D8;   
const int redPin = D0;

MFRC522 rfid(SS_PIN, RST_PIN);
Servo servo;
ESP8266WebServer server(80);
//GLOBAL VARIABLES
bool isGps = false;
bool isFull = false;
struct RFIDLocation {
    String rfid;
    double longitude;
    double latitude;
};

std::map<String, RFIDLocation> locs;
//-----------------------------------------------------
void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    pinMode(redPin, OUTPUT);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    SerialGPS.begin(9600); // Set baud rate for GPS module
    Serial.println("Starting GPS data capture...");

    SPI.begin();
    rfid.PCD_Init();
    servo.attach(SERVO_PIN);
    servo.write(0);
    delay(1000);
}

void loop() {
    getGps();
    checkgps();
    maxCapacity();
    if(isGps){
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        String rfidData = readRFID();
        if (!isRFIDPresent(rfidData)) {
          String balance = getBalanceFromServer(rfidData);
          if (balance =="GOOD"&& !isFull) {
            RFIDLocation newLocation;
            newLocation.rfid = rfidData;
            newLocation.longitude = CurrLongitude;
            newLocation.latitude = CurrLatitude;
            locs[rfidData] = newLocation;
            unlock();
            delay(2000);
            lock();
          }
          else {
            Serial.println("ERROR");
            digitalWrite(redPin, HIGH);
            delay(2000);
            digitalWrite(redPin, LOW);            
          }
          
            
        } else {
          RFIDLocation location = locs[rfidData];
          originlat = location.latitude; // Assign latitude from the location to originlat
          originlong = location.longitude; // Assign longitude from the location to originlong
          Serial.print("Origin: ");
          Serial.println(originlat);
          Serial.println(originlong);
          Serial.print("Destination: ");
          Serial.println(CurrLatitude);
          Serial.println(CurrLongitude);
          calculateAndPrintDistance(originlat, originlong,rfidData);
          unlock();
          delay(2000);
          lock();
        }
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
    }
    }
    
}

//FUNCTIONS
String readRFID() {
    String rfidData;
    for (byte i = 0; i < rfid.uid.size; i++) {
        rfidData += String(rfid.uid.uidByte[i], HEX);
    }
    Serial.println(rfidData);
    return rfidData;
}

void getGps() { //create bool only scan if true
    while (SerialGPS.available() > 0) {
        if (gps.encode(SerialGPS.read())) {
            if (gps.location.isValid()) {
                CurrLatitude = gps.location.lat();
                CurrLongitude = gps.location.lng();
                isGps = true;
            }
            else{
              noGPSBlink();
              isGps = false;
            }
            
        }
    }
}

bool isRFIDPresent(String rfid) {
    return locs.find(rfid) != locs.end();
}

double deg2rad(double deg) {
    return deg * (M_PI / 180);   //degree to radian 
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
               sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = 6371000 * c * 1.32; //multiply by 1.32 for rough estimate 
    //divide by 1000 to convert to KM
    return distance;
}

void calculateAndPrintDistance(double lat1, double lon1,String rfidData) {
    //Destination
    double lat2 = CurrLatitude;
    double lon2 = CurrLongitude;

    // Calculate distance between the origin and the destination
    double distance = calculateDistance(lat1, lon1, lat2, lon2);
    double fare = distance * 2 + 15; //FARE CALCULATION
    // Print the distance
    Serial.print("Fare: $");
    Serial.println(fare, 2); // Print fare with 2 decimal places
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" meters");
    if (sendPostRequest(rfidData, fare)) {
        Serial.println("POST request sent successfully");
        locs.erase(rfidData);
    } else {
        Serial.println("Failed to send POST request");
    }
}
bool sendPostRequest(String rfid, double fare) {
    WiFiClient client;
    HTTPClient http;
    http.begin(client, serverUrl); // Specify the URL of your PHP server
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // Prepare POST data
    String postData = "rfid=" + rfid + "&fare=" + String(fare);

    // Send the POST request
    int httpResponseCode = http.POST(postData);

    // Check for a successful POST request
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("Response: " + response);
        if (response == "success") {
            
            locs.erase(rfid);
            return true;
        }
        else {
        Serial.println("Negative Balance");
        return false;
        }
        
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
        return false;
    }

    http.end(); // Free resources
}
String getBalanceFromServer(String rfid) {
    WiFiClient client;
    HTTPClient http;
    String postData = "rfid=" + rfid; 
    http.begin(client, balUrl); // Specify the URL of your PHP server
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
    // Make the HTTP request
    int httpResponseCode = http.POST(postData);
    if (httpResponseCode == HTTP_CODE_OK) {
        String response = http.getString();
        http.end();
        Serial.println(response);
        return response;
    } else {
        // Handle HTTP error
        http.end();
        return "-1";
    }
}
void checkgps(){
  char inputChar = Serial.read();
    if (inputChar == '1') {
      Serial.println(CurrLatitude);
      Serial.println(CurrLongitude);
    }
}


void unlock() {
  
  for(int i = 0; i <= 180; i += 10) {
    servo.write(i);
  }
}

void lock() {
  for(int i = 180; i >= 0; i -= 10) {
    servo.write(i);
  }
}
void noGPSBlink() {
    for (int i = 0; i < 3; i++) { // Blink the LED 3 times
        digitalWrite(redPin, HIGH);
        delay(500); // Delay for 0.5 seconds
        digitalWrite(redPin, LOW);
        delay(500); // Delay for 0.5 seconds
    }
}
void maxCapacity(){
  int Capacity = locs.size(); 
  if(Capacity == 6){ // change base on PUV CAPACITY
    digitalWrite(redPin, HIGH);
    isFull = true;
  }
  else {
  isFull = false;
  digitalWrite(redPin, LOW);
  }
  
}




