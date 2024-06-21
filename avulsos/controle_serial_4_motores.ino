#include <Arduino.h>
#include <AccelStepper.h>

// Pin definitions for each motor
#define STEP_PIN1 14
#define DIR_PIN1 12

#define IN21 5
#define IN22 18
#define IN23 19
#define IN24 21

#define IN31 26
#define IN32 33
#define IN33 25
#define IN34 32

#define IN41 23
#define IN42 22
#define IN43 4
#define IN44 2

#define LED_BUILTIN 2

// Creating stepper objects
AccelStepper stepper1(1, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::FULL4WIRE, IN21, IN22, IN23, IN24);
AccelStepper stepper3(8, IN31, IN32, IN33, IN34);
AccelStepper stepper4(8, IN41, IN43, IN42, IN44);

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initial motor configurations
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);

  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(500);
}

void loop() {
  if (Serial.available() > 0) {
    String received = Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(received);

    handleCommand(received, stepper1);
    handleCommand(received, stepper2);
    handleCommand(received, stepper3);
    handleCommand(received, stepper4);
  }

  // Execute a step for each motor
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepper4.runSpeed();
}

void handleCommand(String command, AccelStepper &stepper) {
  if (command == "HORARIO") {
    stepper.setSpeed(500); 
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (command == "ANTIHORARIO") {
    stepper.setSpeed(-500); 
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (command == "PARAR") {
    stepper.setSpeed(0); 
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    stepper.setSpeed(0);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
