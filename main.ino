#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <NewPing.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define TRIG_PIN D3  // Trig pin for ultrasonic sensor
#define ECHO_PIN D7 // Echo pin for ultrasonic sensor
#define BUZZER_PIN D0 // Pin for the buzzer
#define GPSRX_SDA D5 // Maximum distance to measure (in cm)
#define GPSTX_SCL D6 // Maximum distance to measure (in cm)
#define MAX_DISTANCE 200 // Maximum distance to measure (in cm)

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

TinyGPSPlus gps;  // The TinyGPS++ object
SoftwareSerial ss(GPSRX_SDA, GPSTX_SCL);  // The serial connection to the GPS device

float latitude, longitude;
int year, month, date, hour, minute, second;
String date_str, time_str, lat_str, lng_str;
int pm;

// Variables for jerk calculation
float prevAccX = 0;    // Previous X-axis acceleration
float prevAccY = 0;    // Previous Y-axis acceleration
float prevAccZ = 0;    // Previous Z-axis acceleration
unsigned long prevTime = 0; // Previous time
unsigned long jerkTimeThreshold = 50; // Threshold to check the jerk in a short time (50ms)

// Dynamic baseline vibration variables
float baselineVibrationZ = 0; // Dynamic baseline for Z-axis vibration
float decayFactor = 0.9; // Decay factor to smooth out baseline changes
float jerkZThreshold = 130.0; // Threshold for Z-axis jerk detection, adjusted to 150 by default

// WiFi credentials
const char* ssid = "Airtel_mant_2587";
const char* password = "Air@43794";

void setup() {
  Serial.begin(9600);  // Initialize serial communication for debugging
  ss.begin(9600);
  Serial.println("Starting...");

  //Connect to WiFi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("Server started");
  // Print the IP address
  Serial.println(WiFi.localIP());

  // Initialize the accelerometer
  if (!accel.begin(0x53)) {
    Serial.println("Failed to initialize ADXL345!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G); // Set range to detect sudden shocks (16G)

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output

  Serial.println("Pothole Detection System with Dynamic Vibration Adjustment Initialized");
  prevTime = millis(); // Initialize time tracking

  // Initialize ESP8266 and connect to WiFi
  // initESP8266();
}

// void initESP8266() {
//   // Send AT commands to ESP8266 to connect to WiFi
//   Serial.println("AT");
//   delay(1000);
  
//   Serial.println("AT+CWMODE=1");  // Set WiFi mode to station mode
//   delay(1000);

//   Serial.print("AT+CWJAP=\"");
//   Serial.print(ssid);
//   Serial.print("\",\"");
//   Serial.print(password);
//   Serial.println("\"");
  
//   delay(5000); // Allow time to connect to WiFi

//   Serial.println("ESP8266 Initialization Complete");
// }

void loop() {

  while (ss.available() > 0){
    if (gps.encode(ss.read())){

      if (gps.location.isValid()){
        latitude = gps.location.lat();
        lat_str = String(latitude, 6);
        longitude = gps.location.lng();
        lng_str = String(longitude, 6);
      }

      if (gps.date.isValid()){
        date_str = "";
        date = gps.date.day();
        month = gps.date.month();
        year = gps.date.year();

        if (date < 10) date_str = '0';
        date_str += String(date);
        date_str += " / ";
        if (month < 10) date_str += '0';
        date_str += String(month);
        date_str += " / ";
        if (year < 10) date_str += '0';
        date_str += String(year);
      }

      if (gps.time.isValid()){
        time_str = "";
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
        minute = (minute + 30);
        if (minute > 59){
          minute = minute - 60;
          hour = hour + 1;
        }

        hour = (hour + 5);
        if (hour > 23) hour = hour - 24;
        if (hour >= 12)
          pm = 1;
        else
          pm = 0;
        hour = hour % 12;
        if (hour < 10) time_str = '0';
        time_str += String(hour);
        time_str += " : ";
        if (minute < 10) time_str += '0';
        time_str += String(minute);
        time_str += " : ";
        if (second < 10) time_str += '0';
        time_str += String(second);
        if (pm == 1)
          time_str += " PM ";
        else
          time_str += " AM ";
      }
      
    }
  }

  sensors_event_t event;
  accel.getEvent(&event);

  // Get current time
  unsigned long currentTime = millis();
  
  // Only calculate jerk if a short time (50ms) has passed
  if (currentTime - prevTime >= jerkTimeThreshold) {
    float dt = (currentTime - prevTime) / 1000.0; // Time interval in seconds

    // Get current accelerations
    float currentAccX = event.acceleration.x;
    float currentAccY = event.acceleration.y;
    float currentAccZ = event.acceleration.z;

    // Update dynamic baseline vibration using a decay factor
    baselineVibrationZ = (baselineVibrationZ * decayFactor) + (abs(currentAccZ) * (1 - decayFactor));

    // Jerk calculation for the X, Y, and Z axes (vertical movement)
    float jerkX = (currentAccX - prevAccX) / dt;
    float jerkY = (currentAccY - prevAccY) / dt;
    float jerkZ = (currentAccZ - prevAccZ) / dt;

    // Print jerk and acceleration values
    Serial.print("Jx : ");
    Serial.print(jerkX);
    Serial.print("\t|\t");
    Serial.print("Jy : ");
    Serial.print(jerkY);
    Serial.print("\t|\t");
    Serial.print("Jz : ");
    Serial.print(jerkZ);
    Serial.print("\t|\t");

    Serial.print("Ax : ");
    Serial.print(currentAccX);
    Serial.print("\t|\t");
    Serial.print("Ay : ");
    Serial.print(currentAccY);
    Serial.print("\t|\t");
    Serial.print("Az : ");
    Serial.print(currentAccZ);
    Serial.print("\t|  ");
    Serial.print((baselineVibrationZ + jerkZThreshold));
    Serial.println();

    // Check for high Z-axis jerk, filtering out baseline vibration
    if (abs(jerkZ) > (baselineVibrationZ + jerkZThreshold)) { // Using jerkZThreshold variable
      Serial.println("Pothole detected by Z-Axis Jerk!");

      // Turn on the buzzer for 50 milliseconds
      digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
      delay(50); // Wait for 50ms
      digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer

      // Measure the pothole depth using the ultrasonic sensor
      // delay(50); // Short delay to stabilize reading
      float depth = sonar.ping_cm();

      if (depth > 0) {
        Serial.println("-----------------------------");
        Serial.print("Pothole Depth: ");
        Serial.print(depth);
        Serial.println(" cm");
        Serial.print("Lat : ");
        Serial.println(lat_str);
        Serial.print("Long : ");
        Serial.println(lng_str);
        Serial.print("Date : ");
        Serial.println(date_str);
        Serial.print("Time : ");
        Serial.println(time_str);
      } else {
        Serial.println("Depth not measurable");
      }

      // Send data to server (lat, long) once pothole is detected
      sendDataToServer(); 
    }

    // Update previous acceleration and time for the next iteration
    prevAccX = currentAccX;
    prevAccY = currentAccY;
    prevAccZ = currentAccZ;
    prevTime = currentTime;
  }

  delay(50); // Small delay to give time for the system to stabilize between loops
}

void sendDataToServer() {
  // Function to send latitude and longitude using ESP8266
  // (You can add this based on GPS integration in the next steps)
}\
