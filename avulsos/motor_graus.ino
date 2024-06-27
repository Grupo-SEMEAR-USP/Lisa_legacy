#include <AccelStepper.h>

#define STEP_PIN1 12
#define DIR_PIN1 14
#define SLEEP_PIN1 13

#define STEP_PIN2 2
#define DIR_PIN2 4
#define SLEEP_PIN2 32

#define passosPorRotacao 200
#define microsteps 1

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

long graus(float graus) {
  return (graus * passosPorRotacao * microsteps) / 360;
}

unsigned long delayStart = 0; // Variável para armazenar o tempo de início do atraso
const unsigned long delayDuration = 1000; // Duração do atraso em milissegundos (1 segundo)
bool stepper2Started = false; // Variável para controlar se o segundo stepper já iniciou

void setup() {
  pinMode(SLEEP_PIN1, OUTPUT);
  pinMode(SLEEP_PIN2, OUTPUT);
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper1.setSpeed(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper2.setSpeed(500);
  stepper1.moveTo(graus(30));
  stepper2.moveTo(graus(90));
  digitalWrite(SLEEP_PIN1, HIGH);
  digitalWrite(SLEEP_PIN2, HIGH);
}

void loop() {
  if (stepper1.distanceToGo() != 0) {
    stepper1.run();
  } else {
    if (!stepper2Started) {
      digitalWrite(SLEEP_PIN1, LOW);
      delayStart = millis(); // Armazena o tempo de início do atraso
      stepper2Started = true; // Indica que o atraso foi iniciado
    }
    if (stepper2Started && (millis() - delayStart >= delayDuration)) {
      if (stepper2.distanceToGo() != 0) {
        stepper2.run();
      } else {
        digitalWrite(SLEEP_PIN2, LOW);
      }
    }
  }
}
