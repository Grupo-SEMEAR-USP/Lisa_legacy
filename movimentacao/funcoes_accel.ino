#include <AccelStepper.h>

// Define a margem de erro como 5 passos
#define erro 0

// Definições de pinos para os motores
// Motor da cabeça - Direita
#define STEP_PIN1 12 
#define DIR_PIN1 14
#define SLEEP_DIREITA 13

// Motor da cabeça - Esquerda
#define STEP_PIN2 2
#define DIR_PIN2  4
#define SLEEP_ESQUERDA 32

// Motor de movimento "não"
#define STEP_PIN3 22
#define DIR_PIN3 23
#define SLEEP_NAO 33

// Motor da base
#define STEP_PIN4 25
#define DIR_PIN4 27 
#define SLEEP_BASE 26

// Motor de movimento "sim"
#define IN41 5
#define IN42 18
#define IN43 19
#define IN44 21

// Definição dos passos por rotação e microsteps
#define passosPorRotacao 200
#define microsteps 1

// Função para converter graus em passos
long graus(float graus) {
  return (graus * passosPorRotacao * microsteps) / 360;
}

// Criação dos objetos stepper para a cabeça e o braço
AccelStepper stepperDIR(1, STEP_PIN1, DIR_PIN1);
AccelStepper stepperESQ(1, STEP_PIN2, DIR_PIN2);
AccelStepper stepperNAO(1, STEP_PIN3, DIR_PIN3);
AccelStepper stepperBASE(1, STEP_PIN4, DIR_PIN4);
AccelStepper stepperSIM(8, IN41, IN43, IN42, IN44);

// Declaração das funções
void configureStepper(AccelStepper &stepper, int sleepPin, float maxSpeed, float acceleration);
void moverAngulo(AccelStepper &stepper, int sleepPin, float angulo, int direcao);
void voltarPara0(AccelStepper &stepper, int sleepPin);
void cumprimentar(AccelStepper &stepperESQ);
void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR);
void sapeca(AccelStepper &stepperBASE, AccelStepper &stepperESQ, AccelStepper &stepperDIR);
void bravinha(AccelStepper &stepperNAO);
void sim(AccelStepper &stepperSIM);
void vitoria(AccelStepper &stepperBASE);
void tristinha(AccelStepper &stepperSIM, AccelStepper &stepperNAO);

void setup() {
  // Inicia a comunicação serial com 115200 baud rate.
  Serial.begin(115200);

  // Configurações dos motores de passo
  configureStepper(stepperDIR, SLEEP_DIREITA, 300, 700);
  configureStepper(stepperESQ, SLEEP_ESQUERDA, 300, 700);
  configureStepper(stepperNAO, SLEEP_NAO, 300, 700);
  configureStepper(stepperBASE, SLEEP_BASE, 300, 500);

  stepperSIM.setMaxSpeed(1000);
  stepperSIM.setAcceleration(500);
}

void loop() {
  cumprimentar(stepperESQ);
}

// Função para configurar um motor de passo
void configureStepper(AccelStepper &stepper, int sleepPin, float maxSpeed, float acceleration) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, LOW);
}

// Função genérica para mover o motor em um ângulo específico
void moverAngulo(AccelStepper &stepper, int sleepPin, float angulo, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(angulo) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved " + String(angulo) + " degrees in direction: " + String(direcao));
}

// Função para retornar o motor à posição 0
void voltarPara0(AccelStepper &stepper, int sleepPin) {
  digitalWrite(sleepPin, HIGH);
  stepper.moveTo(0);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved back to 0 steps");
}

// Função para cumprimentar movendo o motor em 120 graus, variando 30 graus e retornando a 0
void cumprimentar(AccelStepper &stepperESQ) {
  moverAngulo(stepperESQ, SLEEP_ESQUERDA, 120, 1);
  for (int i = 0; i < 2; i++) {
    stepperESQ.setMaxSpeed(500);
    stepperESQ.setAcceleration(1000);
    moverAngulo(stepperESQ, SLEEP_ESQUERDA, 30, 1);  // Horário
    moverAngulo(stepperESQ, SLEEP_ESQUERDA, 30, -1); // Anti-horário
  }
  stepperESQ.setMaxSpeed(300);
  stepperESQ.setAcceleration(700);
  voltarPara0(stepperESQ, SLEEP_ESQUERDA);
  digitalWrite(SLEEP_ESQUERDA, LOW);
}

// Função para executar movimentos de felicidade com dois motores
void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR) {
  for (int i = 0; i < 2; i++) {
    moverAngulo(stepperESQ, SLEEP_ESQUERDA, 120, 1);
    moverAngulo(stepperDIR, SLEEP_DIREITA, 120, -1);
    voltarPara0(stepperESQ, SLEEP_ESQUERDA);
    voltarPara0(stepperDIR, SLEEP_DIREITA);
  }
  digitalWrite(SLEEP_DIREITA, LOW);
  digitalWrite(SLEEP_ESQUERDA, LOW);
}

// Função para executar movimentos de "sapeca"
void sapeca(AccelStepper &stepperSIM, AccelStepper &stepperESQ, AccelStepper &stepperDIR) {
  for (int i = 0; i < 4; i++) {
    moverAngulo(stepperESQ, SLEEP_ESQUERDA, 120, 1);
    moverAngulo(stepperDIR, SLEEP_DIREITA, 120, -1);
    voltarPara0(stepperESQ, SLEEP_ESQUERDA);
    voltarPara0(stepperDIR, SLEEP_DIREITA);
    sim(stepperSIM);
  }
}

// Função para executar movimentos "bravinha"
void bravinha(AccelStepper &stepperNAO) {
  for (int i = 0; i < 2; i++) {
    moverAngulo(stepperNAO, SLEEP_NAO, 60, 1);
    moverAngulo(stepperNAO, SLEEP_NAO, 60, -1);
    moverAngulo(stepperNAO, SLEEP_NAO, 60, -1);
    moverAngulo(stepperNAO, SLEEP_NAO, 60, 1);
  }
  voltarPara0(stepperNAO, SLEEP_NAO);
}

// Função para executar movimentos de "sim"
void sim(AccelStepper &stepperSIM) {
  for (int i = 0; i < 2; i++) {
    stepperSIM.moveTo(1000);
    stepperSIM.runToPosition();
    delay(100);
    stepperSIM.moveTo(0);
    stepperSIM.runToPosition();
    delay(100);
  }
}

// Função para executar movimentos de "vitória"
void vitoria(AccelStepper &stepperBASE) {
  for (int i = 0; i < 4; i++) {
    moverAngulo(stepperBASE, SLEEP_BASE, 90, 1);
    moverAngulo(stepperBASE, SLEEP_BASE, 90, -1);
  }
}

// Função para executar movimentos de "tristinha"
void tristinha(AccelStepper &stepperSIM, AccelStepper &stepperNAO) {
  for (int i = 0; i < 1; i++) {
    stepperSIM.moveTo(1000);
    stepperSIM.runToPosition();
    delay(3000);
    moverAngulo(stepperNAO, SLEEP_NAO, 60, 1);
    moverAngulo(stepperNAO, SLEEP_NAO, 60, -1);
    delay(1000);
  }
}
