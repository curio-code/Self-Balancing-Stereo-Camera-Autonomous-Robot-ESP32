#define min _min
#define max _max

//L298N LOGIC PINS
#define SX_1 9
#define SX_2 8
#define DX_1 5
#define DX_2 4

#define SX_PWM 10
#define DX_PWM 3

// PID Constants
float kp = 150.0f;    // Proportional coefficient 400 - 6V 300.0f  200.0
float ki = 300.0f;    // Integral coefficient      10 - 6V 10.0f   0.0
float kd = 0.0f;      // Derivative coefficient     3 - 6V 2.5f    1.0
float setpoint = 0.5f;    // Desired angle, corresponding to the vertical line.
const float i_limit = 250.0f;   // Integral part limit in absolute value

// PID Variables
float prevError = 0.0f;
float proportional = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;


float updatePID(float angle, float dt) {
  float error = setpoint - angle;

  proportional = kp * error;
  integral += ki * error * dt;
  integral = constrain(integral, -i_limit, i_limit); // Limit the integrated error - prevents windup
  derivative = kd * (error - prevError) / dt;
  float output = proportional + integral + derivative;
  //Serial.println(proportional);
  prevError = error;
  return constrain(output, -250, 250);
}

void runMotors(float pidOutput) {
  /*
   * L298N Configuration Table
   * Direction of rotation for the motors wrt input logic
   *
   *  In1  |  In2  | Evento
   * ------|-------|----------------------
   *  LOW  |  LOW  | stop
   *  LOW  |  HIGH | one direction
   *  HIGH |  LOW  | the other direction
   *  HIGH |  HIGH | stop
   */

  // Determining the direction of rotation based on pidOutput sign
  if(pidOutput < 0) {
    digitalWrite(SX_1, LOW);
    digitalWrite(SX_2, HIGH);
    digitalWrite(DX_1, HIGH);
    digitalWrite(DX_2, LOW);
  } else {
    digitalWrite(SX_1, HIGH);
    digitalWrite(SX_2, LOW);
    digitalWrite(DX_1, LOW);
    digitalWrite(DX_2, HIGH);
  }

  // Computing speed power
  float speed = min(abs(pidOutput), 255);

  analogWrite(SX_PWM,speed);
  analogWrite(DX_PWM,speed);
}

void initMotors() {
  pinMode(SX_1, OUTPUT);
  pinMode(SX_2, OUTPUT);
  pinMode(DX_1, OUTPUT);
  pinMode(DX_2, OUTPUT);
  pinMode(SX_PWM, OUTPUT);
  pinMode(DX_PWM, OUTPUT);

  digitalWrite(SX_1, LOW);
  digitalWrite(SX_2, LOW);
  digitalWrite(DX_1, LOW);
  digitalWrite(DX_2, LOW);
  digitalWrite(SX_PWM, LOW);
  digitalWrite(DX_PWM, LOW);
}

void tunePID (float newKp, float newKi, float newKd){
    kp = newKp;
    ki = newKi;
    kd = newKd;
}

void stopMotors() {
  digitalWrite(SX_PWM, LOW);
  digitalWrite(DX_PWM, LOW);
}
