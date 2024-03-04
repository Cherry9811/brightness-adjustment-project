#include <NewPing.h>

#define TRIGGER_PIN 2  // Define the trigger pin
#define ECHO_PIN 4     // Define the echo pin
#define LED_PIN 5      // Define the LED pin

NewPing sonar(TRIGGER_PIN, ECHO_PIN);  // Create a NewPing object

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);  // Initialize LED pin as an output
}

void loop() {
  unsigned int distance = sonar.ping_cm();  // Get distance in centimeters

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Control the LED based on the measured distance
  if (distance < 100) {  // If the distance is less than 100 cm, turn on the LED
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);  // Otherwise, turn off the LED
  }

  delay(1000);  // Wait for 1 second before taking another measurement
}
