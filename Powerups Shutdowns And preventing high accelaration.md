// Define the motor pin
int motorPin = 9;

// Define the initial motor speed and acceleration
float initialSpeed = 0.0;
float acceleration = 0.01;

void setup() {
  // Initialize the motor pin as an output
  pinMode(motorPin, OUTPUT);

  // Set the motor speed to the initial speed
  analogWrite(motorPin, initialSpeed);
}

void loop() {
  // Check if the motor speed is below the maximum speed
  if (analogRead(A0) < 1023) {
    // Calculate the new motor speed based on the acceleration
    float newSpeed = initialSpeed + acceleration;

    // Prevent sudden high accelerations
    if (newSpeed - initialSpeed > 0.1) {
      newSpeed = initialSpeed + 0.1;
    }

    // Set the motor speed to the new speed
    analogWrite(motorPin, newSpeed);

    // Update the initial speed
    initialSpeed = newSpeed;
  }
  else {
    // Stop the motor
    analogWrite(motorPin, 0);

    // Reset the initial speed to 0
    initialSpeed = 0.0;

    // Wait for 1 second
    delay(1000);
  }
}

