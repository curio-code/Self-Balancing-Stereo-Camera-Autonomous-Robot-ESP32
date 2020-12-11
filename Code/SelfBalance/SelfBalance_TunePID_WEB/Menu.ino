void setup_menu() {
  bool exit_menu = false;
  while (Serial.available()) Serial.read();
  while (not exit_menu) {
    Serial.println(F("CONFIGURATION MENU"));
    Serial.println(F("## Kalman Parameters"));
    Serial.print(F("[1]\tModify V1_angle\t\t[Actual: ")); Serial.print(KFilter.V1_angle, 3); Serial.println(F("]"));
    Serial.print(F("[2]\tModify V1_bias\t\t[Actual: ")); Serial.print(KFilter.V1_bias, 3); Serial.println(F("]"));
    Serial.print(F("[3]\tModify V2_measure\t[Actual: ")); Serial.print(KFilter.V2_measure, 3); Serial.println(F("]"));
    Serial.println(F("## PID Parameters"));
    Serial.print(F("[4]\tModify kp\t\t[Actual: ")); Serial.print(kp); Serial.println(F("]"));
    Serial.print(F("[5]\tModify ki\t\t[Actual: ")); Serial.print(ki); Serial.println(F("]"));
    Serial.print(F("[6]\tModify kd\t\t[Actual: ")); Serial.print(kd); Serial.println(F("]"));
    Serial.print(F("[7]\tModify setpoint\t\t[Actual: ")); Serial.print(setpoint); Serial.println(F("]"));
    Serial.println(F("## General Setup"));
    Serial.println(F("[8]\tExit"));
    Serial.println(F("Choose the operation: "));

    while (not Serial.available());
    char choice = Serial.read();

    float value;

    switch (choice) {
      case 49:
        Serial.println(F("Insert a new value for V1_angle: "));
        while (not Serial.available());
        value = Serial.parseFloat();
        KFilter.V1_angle = value;
        break;
      case 50:
        Serial.println(F("Insert a new value for V1_bias: "));
        while (not Serial.available());
        value = Serial.parseFloat();
        KFilter.V1_bias = value;
        break;
      case 51:
        Serial.println(F("Insert new value for V2_measure: "));
        while (not Serial.available());
        value = Serial.parseFloat();
        KFilter.V2_measure = value;
        break;
      case 52:
        Serial.println(F("Insert new value for kp: "));
        while (not Serial.available());
        value = Serial.parseFloat();
        kp = value;
        break;
      case 53:
        Serial.println(F("Insert new value for ki: "));
        while (not Serial.available());
        value = Serial.parseFloat();
        ki = value;
        break;
      case 54:
        Serial.println(F("Insert new value for kd: "));
        while (not Serial.available());
        value = Serial.parseFloat();
        kd = value;
        break;
      case 55:
        Serial.println(F("Insert new value for setpoint: "));
        while (not Serial.available());
        value = Serial.parseFloat();
        setpoint = value;
        break;
      case 56:
        exit_menu = true;
        break;
      default:
        Serial.println(F("Invalid command!"));
        break;
    }
  }
}
