// Define motor control pins
const int MOTOR_DIR_PIN = 2;
const int MOTOR_PWM_PIN = 3;

// Define sensor pins
const int LIMIT_SWITCH_PIN = 4;
const int ACCELEROMETER_X_PIN = A0;
const int ACCELEROMETER_Y_PIN = A1;

// Define motor control parameters
const int MAX_PWM_VALUE = 255;
const int MIN_PWM_VALUE = 0;
const int MAX_ACCELERATION = 100; // in mm/s^2
const int MAX_VELOCITY = 200; // in mm/s
const int HOME_POSITION = 0; // in mm
const int MAX_POSITION = 500; // in mm

// Define safety parameters
const int ACCELERATION_THRESHOLD = 500; // in mm/s^2
const int EMERGENCY_STOP_THRESHOLD = 200; // in mm/s

// Define global variables
int currentPosition = HOME_POSITION;
int currentVelocity = 0;
bool isMoving = false;

// Define motor control functions
void setMotorDirection(int direction) {
  digitalWrite(MOTOR_DIR_PIN, direction);
}

void setMotorPWM(int pwmValue) {
  analogWrite(MOTOR_PWM_PIN, pwmValue);
}

void moveMotorToPosition(int position) {
  isMoving = true;
  int direction = (position > currentPosition) ? HIGH : LOW;
  setMotorDirection(direction);
  
  while (currentPosition != position) {
    int distanceToGo = abs(position - currentPosition);
    int acceleration = min(MAX_ACCELERATION, distanceToGo);
    int velocity = min(MAX_VELOCITY, sqrt(2 * acceleration * distanceToGo));
    int pwmValue = map(velocity, 0, MAX_VELOCITY, MIN_PWM_VALUE, MAX_PWM_VALUE);
    
    if (velocity < currentVelocity) {
      delay(10);
      continue;
    }
    
    if (abs(velocity - currentVelocity) > ACCELERATION_THRESHOLD) {
      isMoving = false;
      emergencyStop();
      return;
    }
    
    setMotorPWM(pwmValue);
    currentVelocity = velocity;
    
    if (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
      currentPosition = HOME_POSITION;
      currentVelocity = 0;
      isMoving = false;
      break;
    }
    
    currentPosition += direction;
    delay(10);
  }
  
  isMoving = false;
}

void emergencyStop() {
  setMotorPWM(MIN_PWM_VALUE);
  while (currentVelocity > EMERGENCY_STOP_THRESHOLD) {
    currentVelocity = currentVelocity / 2;
    delay(10);
  }
  currentVelocity = 0;
}

// Define sensor reading functions
int readAccelerometerX() {
  return analogRead(ACCELEROMETER_X_PIN);
}

int readAccelerometerY() {
  return analogRead(ACCELEROMETER_Y_PIN);
}

// Define setup function
void setup() {
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT);
  
  // Initialize motor control pins
  setMotorDirection(HIGH);
  setMotorPWM(MIN_PWM_VALUE);
  
  // Print initial status
  Serial.begin(9600);
  Serial.println("System initialized.");
}

// Define main loop
void loop() {
  if (isMoving) {
    int accelerationX = readAccelerometerX();
    int accelerationY = readAccelerometerY();
    
    if (abs(accelerationX) > ACCELERATION_THRESHOLD || abs(accelerationY) > ACCELERATION_THRESHOLD) {
      isMoving = false;
      emergencyStop();
    }
  }
}
