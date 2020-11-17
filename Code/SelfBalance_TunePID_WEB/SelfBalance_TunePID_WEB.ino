#include <analogWrite.h>
#include <Wire.h>
#include <Math.h>
#include <Ticker.h>
#include <MPU6050.h>
#include "Kalman.h"
#include "Config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "JioFiber-xeeV3";
const char* password = "shail817111";

// FUNCTIONS DECLARATION
void blinkLed();
void turnOnLed();
void turnOffLed();
void calibrate();
void readSensor();
float getAccAngle(float, float);

// EXTERNAL VARIABLES DECLARATION
extern float kp; extern float proportional;
extern float ki; extern float integral;
extern float kd; extern float derivative;
extern float setpoint;

const char* PARAM_Kp = "inputKp";
const char* PARAM_Ki = "inputKi";
const char* PARAM_Kd = "inputKd";

float Kp=kp, Ki=ki, Kd=kd; // Capital Kp, Ki, Kd are the values avaibale from WebServer
float prevKp=Kp, prevKi=Ki, prevKd=Kd;

// HTML web page to handle 3 input fields (inputKp, inputKi, inputKd)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <h1>Online PID Tuning</h1>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      setTimeout(function(){ document.location.reload(false); }, 500);
    }
  </script></head><body>
  <form action="/get" target="hidden-form">
    inputKp (current value %inputKp%): <input type="number" step="0.0001" name="inputKp">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    inputKi (current value %inputKi%): <input type="number" step="0.0001" name="inputKi">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    inputKd (current value %inputKd%): <input type="number" step="0.0001" name="inputKd">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form>
  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// Replaces placeholder with stored values
String processor(const String& var){
  //Serial.println(var);
  if(var == "inputKp"){
    return String(kp);
  }
  else if(var == "inputKi"){
    return String(ki);
  }
  else if(var == "inputKd"){
    return String(kd);
}
  return String();
}

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int32_t ax_offset, ay_offset, az_offset;
int32_t gx_offset, gy_offset, gz_offset;

// Stores the starting position of the robot when it is powered on
// true when x is poiting upwards, false when x is pointing downwards
bool startPositionIsUp;

// Time of the last call to kalman function, using microseconds precision
unsigned long oldTime;

// Kalman instance used to filter IMU sensor readings
Kalman KFilter;

// LED variables and ticker used for blink
bool ledState = 0;
Ticker ledTicker(blinkLed, 100, 0, MILLIS);


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(38400);

  #if PRINT_INFO
  Serial.println(F("Initializing L298N motor driver..."));
  #endif
  initMotors();

  // initialize device
  //Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  #if PRINT_INFO
  Serial.println(F("Testing device connections..."));
  #endif
  bool conn_ok = accelgyro.testConnection();
  #if PRINT_INFO
  Serial.println(conn_ok ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  #endif
  if (not conn_ok) while(1);

  // configure Arduino LED pin for output
  pinMode(LED_BUILTIN, OUTPUT);
  ledTicker.start(); // we signal the beginning of the calibration procedure

  // calibration procedure
  #if PRINT_INFO
  Serial.println(F("Calibrating in 3 seconds..."));
  #endif
  delay(3000);
  calibrate();

  ledTicker.stop(); // calibration procedure ended
  turnOffLed();

  // initializing kalman object
  #if PRINT_INFO
  Serial.println(F("Initializing Kalman Filter..."));
  #endif
  KFilter = Kalman();
  float startAngle = (startPositionIsUp == true) ? +90.0f : -90.0f;
  KFilter.setAngle(startAngle);

  #if ENABLE_CONFIG
  Serial.println(F("Press any key to enter the configuration menu..."));
  delay(3000);
  if (Serial.available()) {
    setup_menu();
  }
  #endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputKp=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET inputKp value on <ESP_IP>/get?inputKp=<inputMessage>
    if (request->hasParam(PARAM_Kp)) {
      inputMessage = request->getParam(PARAM_Kp)->value();
      Kp = inputMessage.toFloat();
    }
    // GET inputKi value on <ESP_IP>/get?inputKi=<inputMessage>
    else if (request->hasParam(PARAM_Ki)) {
      inputMessage = request->getParam(PARAM_Ki)->value();
      Ki = inputMessage.toFloat();
    }
    // GET inputKd value on <ESP_IP>/get?inputKd=<inputMessage>
    else if (request->hasParam(PARAM_Kd)) {
      inputMessage = request->getParam(PARAM_Kd)->value();
      Kd = inputMessage.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });

  server.onNotFound(notFound);
  server.begin();

  // initializing oldTime with the current time
  oldTime = micros();

  #if PRINT_INFO
  Serial.println(F("Setup done."));
  #endif

  turnOnLed();
}

void loop() {
  if ((prevKp!=Kp)||(prevKi!=Ki)||(prevKd!=Kd)){
      tunePID(Kp, Ki, Kd);
      prevKp=Kp; prevKi=Ki; prevKd=Kd;
  }
  // acquiring current time and computing delta time wrt the previous iteration
  // in seconds
  unsigned long time = micros();
  unsigned long dt_usec = time - oldTime;
  float dt_sec = float(dt_usec) / 1000000.0f;

  // reading new values from IMU
  readSensor();
  // converting raw readings in physical measures
  float gyroYRate = float(gy) / 131.072f;       // deg/s
  float accX = (float(ax) * 9.81f) / 16384.0f;  // m/s^2
  float accZ = (float(az) * 9.81f) / 16384.0f;  // m/s^2

  float accAngle = getAccAngle(accX, accZ);

  // calling the Kalman filter update function
  float kalmanAngle = KFilter.getAngle(accAngle, gyroYRate, dt_sec);

  // PID & Motor Control
  float pidOutput = updatePID(kalmanAngle, dt_sec);
  // enabling motors only if the robot is near the vertical position
  if (abs(kalmanAngle) < 30) {
    runMotors(pidOutput);
  } else {
    stopMotors();
  }
  // printing values for plotter
  #if PRINT_DATA
  //Serial.print(gyroYRate); Serial.print('\t');                    // gyroscope y-rate
  //Serial.print(accX); Serial.print('\t');                         // acceleration along x-axis (m/s^2)
  //Serial.print(accZ); Serial.print('\t');                         // acceleration along z-axis (m/s^2)
  //Serial.print(pidOutput); Serial.print('\t');                     // angle computed from accelerometer (degrees)
  //Serial.print(KFilter.bias, 20); Serial.print('\t');             // Kalman bias
  //Serial.print(KFilter.rate, 20); Serial.print('\t');             // Kalman rate
  //Serial.print(KFilter.e, 20); Serial.print('\t');                // Kalman innovation term
  //Serial.print(kalmanAngle); Serial.print('\t');                  // Kalman filtered angle
  //Serial.print(proportional); Serial.print('\t');                 // Proportional component of PID
  //Serial.print(integral); Serial.print('\t');                     // Integral component of PID
  //Serial.print(derivative); Serial.print('\t');                   // Derivative component of PID
  //Serial.print(pidOutput/10); Serial.print('\t');                 // PID output (normalized between -25 and +25)
  //Serial.print(dt_usec); Serial.print('\t');                      // duration of the cycle in milliseconds

  Serial.println();
  #endif

  oldTime = time;

  delay(15);
}

//// Functions
//Switch led state
void blinkLed() {
  digitalWrite(LED_BUILTIN, ledState);
  ledState = !ledState;
}

// Turn on EPS32 board's led
void turnOnLed() {
  digitalWrite(LED_BUILTIN, HIGH);
  ledState = 1;
}

// Turn off EPS32 board's led
void turnOffLed() {
  digitalWrite(LED_BUILTIN, LOW);
  ledState = 0;
}

/* Calibration procedure. To determine the offset of accelerometer and gyroscope,
 * we compute the mean of a bunch of initial readings obtained with the sensor in a fixed position.*/
void calibrate() {
  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < 512; i++) {
    ledTicker.update();
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;

    delay(5);
  }
  // dividing by 512
  ax_offset = ax_sum>>9; ay_offset = ay_sum>>9; az_offset = az_sum>>9;
  gx_offset = gx_sum>>9; gy_offset = gy_sum>>9; gz_offset = gz_sum>>9;

  // if the mpu6050 is positioned with x-axis pointing upwards, ax should be equal to 16384
  // here we chech the sign of ax in order to determine in which of the two initial positions
  // the robot is lying.

  if (ax_offset > 0) {  // x-axis pointing upwards
    ax_offset -= 16384;
    startPositionIsUp = true;
  } else {              // x-axis pointing downwards
    ax_offset += 16384;
    startPositionIsUp = false;
  }

  // printing calibration values
  #if PRINT_INFO
  Serial.print(F("\t Acc offsets:\t")); Serial.print(ax_offset);
  Serial.print(F("\t")); Serial.print(ay_offset);
  Serial.print(F("\t")); Serial.println(az_offset);
  Serial.print(F("\tGyro offsets:\t")); Serial.print(gx_offset);
  Serial.print(F("\t")); Serial.print(gy_offset);
  Serial.print(F("\t")); Serial.println(gz_offset);
  #endif

}

// Read current values from sensor and apply offset correction.
void readSensor() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // appliyng offsets
  ax -= ax_offset; ay -= ay_offset; az -= az_offset;
  gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;
}

/* Function that computes the robot angle from the accelerometer values of x and z.
 * - Parameters:
 *      - accX  : the accelerometer measurement for the x axis [in degrees]
 *      - accZ  : the accelerometer measurement for the z axis [in degrees]
 * - Returns: the value of the robot angle with respect to the vertical position [0 degrees when vertical].*/
float getAccAngle(float accX, float accZ) {
  float angle = (atan2(accX, -accZ)) * RAD_TO_DEG;
  return angle;
}
