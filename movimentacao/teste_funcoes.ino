#include <AccelStepper.h>

#define erro 0  // Define a margem de erro como 5 passos

// Definições de pinos para o motor da cabeça
// DIREITA
#define STEP_PIN1 12 
#define DIR_PIN1 14
#define SLEEP_PIN1 13

// ESQUERDA
#define STEP_PIN2 2
#define DIR_PIN2 4
#define SLEEP_PIN2 32

// NAO
#define STEP_PIN3 23
#define DIR_PIN3 21
#define SLEEP_PIN3 33

// BASE
#define STEP_PIN4 25
#define DIR_PIN4 27
#define SLEEP_PIN4 26

// SIM
#define IN41 5
#define IN42 18
#define IN43 19
#define IN44 21

#define passosPorRotacao 200
#define microsteps 1

long graus(float graus) {
  return (graus * passosPorRotacao * microsteps) / 360;
}

// Criação dos objetos stepper para a cabeça e o braço
AccelStepper stepperDIR(1, STEP_PIN1, DIR_PIN1);
AccelStepper stepperESQ(1, STEP_PIN2, DIR_PIN2);
AccelStepper stepperNAO(1, STEP_PIN3, DIR_PIN3);
AccelStepper stepperBASE(1, STEP_PIN4, DIR_PIN4);
AccelStepper stepperSIM(8, IN41, IN43, IN42, IN44);

void setup() {
  // Inicia a comunicação serial com 115200 baud rate.
  Serial.begin(115200);

  // Configurações dos motores de passo
  configureStepper(stepperDIR, SLEEP_PIN1, 300, 500);
  configureStepper(stepperESQ, SLEEP_PIN2, 300, 500);
  configureStepper(stepperNAO, SLEEP_PIN3, 1000, 500);
  configureStepper(stepperBASE, SLEEP_PIN4, 1000, 500);
}

void loop() {
  //cumprimentar(stepperDIR);
  feliz(stepperESQ,stepperDIR);
}

void configureStepper(AccelStepper &stepper, int sleepPin, float maxSpeed, float acceleration) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, LOW);
}

void virar120(AccelStepper &stepper, int sleepPin, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(120) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved 60 steps in direction: " + String(direcao));
}
void virar90(AccelStepper &stepper, int sleepPin, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(90) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved 60 steps in direction: " + String(direcao));
}
void virar60(AccelStepper &stepper, int sleepPin, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(60) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved 60 steps in direction: " + String(direcao));
}
void virar45(AccelStepper &stepper, int sleepPin, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(45) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved 60 steps in direction: " + String(direcao));
}
void virar30(AccelStepper &stepper, int sleepPin, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(30) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved 30 steps in direction: " + String(direcao));
}
void virar15(AccelStepper &stepper, int sleepPin, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(15) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved 60 steps in direction: " + String(direcao));
}

void voltarPara0(AccelStepper &stepper, int sleepPin) {
  digitalWrite(sleepPin, HIGH);
  stepper.moveTo(0);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved back to 0 steps");
}

void cumprimentar(AccelStepper &stepperDIR) {
  // Move para 60 graus (horário)
  virar120(stepperDIR, SLEEP_PIN1, 1);

  // Variar 30 graus (horário e anti-horário) 4 vezes
  for (int i = 0; i < 4; i++) {
    virar30(stepperDIR, SLEEP_PIN1,1);  // Horário
    virar30(stepperDIR, SLEEP_PIN1,-1); // Anti-horário
  }

  // Voltar para 0 (anti-horário)
  voltarPara0(stepperDIR, SLEEP_PIN1);
}

void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR) {

  for (int i = 0; i < 4; i++) {
    virar120(stepperESQ, SLEEP_PIN2, 1);
    virar120(stepperDIR, SLEEP_PIN1, -1);
    voltarPara0(stepperESQ, SLEEP_PIN2);
    voltarPara0(stepperDIR, SLEEP_PIN1);

  }
}

void sapeca(AccelStepper &stepperBASE, AccelStepper &stepperESQ, AccelStepper &stepperDIR) {

  for (int i = 0; i < 4; i++) {
    virar120(stepperESQ, SLEEP_PIN2, 1);
    virar120(stepperDIR, SLEEP_PIN1, -1);
    virar60(stepperBASE, SLEEP_PIN4,1);  // Horário
    virar60(stepperBASE, SLEEP_PIN4,-1); // Anti-horário
    voltarPara0(stepperESQ, SLEEP_PIN2);
    voltarPara0(stepperDIR, SLEEP_PIN1);

  }
  voltarPara0(stepperBASE, SLEEP_PIN4);
}

void bravinha(AccelStepper &stepperNAO){
  for (int i = 0; i < 4; i++) {
    virar60(stepperNAO, SLEEP_PIN3, 1);
    virar60(stepperNAO, SLEEP_PIN3, -1);
  }
}

void sim(AccelStepper &stepperSIM){
  for (int i = 0; i < 4; i++) {
    stepperSIM.moveTo(1000);
    stepperSIM.runToPosition();
    delay(1000);
    stepperSIM.moveTo(0);
    stepperSIM.runToPosition();
    delay(1000);
  }
}

void vitoria(AccelStepper &stepperBASE){
  for (int i = 0; i < 4; i++) {
    virar90(stepperBASE, SLEEP_PIN4, 1);
    virar90(stepperBASE, SLEEP_PIN4, -1);
  }
}