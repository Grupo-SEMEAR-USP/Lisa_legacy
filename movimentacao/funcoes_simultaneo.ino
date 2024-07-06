#include <AccelStepper.h>

// Define a margem de erro como 5 passos
#define erro 0

// Definições de pinos para os motores
// DIREITA
#define STEP_PIN1 12 
#define DIR_PIN1 14
#define SLEEP_DIREITA 13

// ESQUERDA
#define STEP_PIN2 2
#define DIR_PIN2  4
#define SLEEP_ESQUERDA 32 

// NAO
#define STEP_PIN3 22
#define DIR_PIN3 23
#define SLEEP_NAO 33

// BASE
#define STEP_PIN4 25
#define DIR_PIN4 27 
#define SLEEP_BASE 26

// SIM
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

// Definição dos passos por rotação e microsteps
#define passosPorRotacaoSIM 4096
#define microstepsSIM 1

// Função para converter graus em passos
long grausSIM(float graus) {
  return (graus * passosPorRotacaoSIM * microstepsSIM) / 360;
}

// Criação dos objetos stepper para a cabeça e o braço
AccelStepper stepperDIR(1, STEP_PIN1, DIR_PIN1);
AccelStepper stepperESQ(1, STEP_PIN2, DIR_PIN2);
AccelStepper stepperNAO(1, STEP_PIN3, DIR_PIN3);
AccelStepper stepperBASE(1, STEP_PIN4, DIR_PIN4);
AccelStepper stepperSIM(8, IN41, IN43, IN42, IN44);

void configureStepper(AccelStepper &stepper, int sleepPin, float maxSpeed, float acceleration);
void virar(AccelStepper &stepper, int sleepPin, float angle, int direcao);
void voltarPara0(AccelStepper &stepper, int sleepPin);
void cumprimentar(AccelStepper &stepperESQ);
void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR);
void sapeca(AccelStepper &stepperSIM, AccelStepper &stepperNAO, AccelStepper &stepperESQ, AccelStepper &stepperDIR);
void bravinha(AccelStepper &stepperNAO);
void sim(AccelStepper &stepperSIM);
void vitoria(AccelStepper &stepperBASE);
void tristinha(AccelStepper &stepperSIM, AccelStepper &stepperNAO);
void confusa(AccelStepper &stepperNAO, AccelStepper &stepperSIM);

void setup() {
  // Inicia a comunicação serial com 115200 baud rate.
  Serial.begin(115200);

  // Configurações dos motores de passo
  configureStepper(stepperDIR, SLEEP_DIREITA, 300, 700);
  configureStepper(stepperESQ, SLEEP_ESQUERDA, 300, 700);
  configureStepper(stepperNAO, SLEEP_NAO, 300, 700);
  configureStepper(stepperBASE, SLEEP_BASE, 300, 500);
  configureStepper(stepperSIM, 0, 1000, 500); // SLEEP_PIN não é usado para o stepperSIM
}

void loop() {
  //sapeca(stepperSIM, stepperESQ, stepperDIR);
  //bravinha(stepperNAO);
  sapeca(stepperSIM,stepperNAO, stepperESQ,stepperDIR);
  //cumprimentar(stepperESQ);
  //confusa(stepperNAO,stepperSIM);
  //feliz(stepperESQ,stepperDIR);
}

void configureStepper(AccelStepper &stepper, int sleepPin, float maxSpeed, float acceleration) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  if (sleepPin != 0) {
    pinMode(sleepPin, OUTPUT);
    digitalWrite(sleepPin, LOW);
  }
}

void virar(AccelStepper &stepper, int sleepPin, float angle, int direcao) {
  if (sleepPin != 0) {
    digitalWrite(sleepPin, HIGH);
  }
  stepper.move(graus(angle) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moveu " + String(angle) + " graus na direcao: " + String(direcao));
}

void voltarPara0(AccelStepper &stepper, int sleepPin) {
  if (sleepPin != 0) {
    digitalWrite(sleepPin, HIGH);
  }
  stepper.moveTo(0);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor zerado");
  if (sleepPin != 0) {
    digitalWrite(sleepPin, LOW);
  }
}

void cumprimentar(AccelStepper &stepperESQ) {
  virar(stepperESQ, SLEEP_ESQUERDA, 120, 1);
  for (int i = 0; i < 2; i++) {
    stepperESQ.setMaxSpeed(500);
    stepperESQ.setAcceleration(1000);
    virar(stepperESQ, SLEEP_ESQUERDA, 30, 1);
    virar(stepperESQ, SLEEP_ESQUERDA, 30, -1);
  }
  stepperESQ.setMaxSpeed(300);
  stepperESQ.setAcceleration(700);
  voltarPara0(stepperESQ, SLEEP_ESQUERDA);
  Serial.println("Cumprimentando...");
}

void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR) {
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  for (int i = 0; i < 2; i++) {
    stepperESQ.setMaxSpeed(100);
    stepperESQ.setAcceleration(300);
    stepperDIR.setMaxSpeed(100);
    stepperDIR.setAcceleration(300);

    stepperESQ.moveTo(graus(120));
    stepperDIR.moveTo(graus(120));

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
    }

    stepperESQ.moveTo(0);
    stepperDIR.moveTo(0);

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
    }
  }
  stepperESQ.setMaxSpeed(300);
  stepperESQ.setAcceleration(500);
  stepperDIR.setMaxSpeed(300);
  stepperDIR.setAcceleration(500);  
  digitalWrite(SLEEP_DIREITA, LOW);
  digitalWrite(SLEEP_ESQUERDA, LOW);
  Serial.println("LISA feliz...");
}

void sapeca(AccelStepper &stepperSIM, AccelStepper &stepperNAO, AccelStepper &stepperESQ, AccelStepper &stepperDIR) {
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  digitalWrite(SLEEP_NAO, HIGH);

  stepperNAO.setMaxSpeed(100);
  stepperNAO.setAcceleration(500);
  stepperESQ.setMaxSpeed(200);
  stepperESQ.setAcceleration(500);
  stepperDIR.setMaxSpeed(200);
  stepperDIR.setAcceleration(300);
  stepperSIM.setMaxSpeed(1000);
  stepperSIM.setAcceleration(1000);

  for (int i = 0; i < 2; i++) {
    // Movimenta os steppers para as posições alvo iniciais
    stepperESQ.moveTo(graus(60));
    stepperDIR.moveTo(-graus(120));
    stepperSIM.moveTo(grausSIM(60));
    stepperNAO.moveTo(graus(60));


    // Loop enquanto qualquer um dos motores ainda estiver se movendo
    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperSIM.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperSIM.run();
      stepperNAO.run();
    }

    // Movimenta os steppers para as novas posições alvo
    stepperESQ.moveTo(0);
    stepperDIR.moveTo(0);
    stepperSIM.moveTo(0);
    stepperNAO.moveTo(0);

    // Loop enquanto qualquer um dos motores ainda estiver se movendo
    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperSIM.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperSIM.run();
      stepperNAO.run();
    }
  }
  

  digitalWrite(SLEEP_DIREITA, LOW);
  digitalWrite(SLEEP_ESQUERDA, LOW);
  digitalWrite(SLEEP_NAO,LOW);
  Serial.println("LISA sapeca...");
}

void bravinha(AccelStepper &stepperNAO) {
  for (int i = 0; i < 2; i++) {
    virar(stepperNAO, SLEEP_NAO, 60, 1);
    virar(stepperNAO, SLEEP_NAO, 60, -1);
    virar(stepperNAO, SLEEP_NAO, 60, -1);
    virar(stepperNAO, SLEEP_NAO, 60, 1);
  }
  voltarPara0(stepperNAO, SLEEP_NAO);
  Serial.println("LISA bravinha...");
}

void confusa(AccelStepper &stepperNAO, AccelStepper &stepperSIM) {
  digitalWrite(SLEEP_NAO, HIGH);

  stepperNAO.setMaxSpeed(100);
  stepperNAO.setAcceleration(500);
  stepperSIM.setMaxSpeed(500);
  stepperSIM.setAcceleration(1000);

  // Configura o SIM para girar continuamente
  stepperSIM.setSpeed(500);  // Configura a velocidade do SIM

  for (int i = 0; i < 2; i++) {
    // Movimenta o stepperNAO para a posição alvo inicial
    stepperNAO.moveTo(graus(60));

    // Loop enquanto qualquer um dos motores ainda estiver se movendo
    while (stepperNAO.distanceToGo() != 0) {
      stepperNAO.run();
      stepperSIM.runSpeed();  // Gira o SIM continuamente
    }

    // Movimenta o stepperNAO para a nova posição alvo
    stepperNAO.moveTo(0);

    // Loop enquanto qualquer um dos motores ainda estiver se movendo
    while (stepperNAO.distanceToGo() != 0) {
      stepperNAO.run();
      stepperSIM.runSpeed();  // Gira o SIM continuamente
    }
  }

  digitalWrite(SLEEP_NAO, LOW);
}

void sim(AccelStepper &stepperSIM) {
  for (int i = 0; i < 2; i++) {
    stepperSIM.setMaxSpeed(100);
    stepperSIM.setAcceleration(100);

    stepperSIM.moveTo(1000);
    stepperSIM.runToPosition();
    delay(100);
    stepperSIM.moveTo(0);
    stepperSIM.runToPosition();
    delay(100);
  }
  Serial.println("LISA confirmando...");
}

void vitoria(AccelStepper &stepperBASE) {
  for (int i = 0; i < 4; i++) {
    virar(stepperBASE, SLEEP_BASE, 90, 1);
    virar(stepperBASE, SLEEP_BASE, 90, -1);
  }
  digitalWrite(SLEEP_BASE, LOW);
  Serial.println("LISA vitoriosa...");
}

void tristinha(AccelStepper &stepperSIM, AccelStepper &stepperNAO) {
  for (int i = 0; i < 1; i++) {
    stepperSIM.setMaxSpeed(100);
    stepperSIM.setAcceleration(100);

    stepperSIM.moveTo(1000);
    stepperSIM.runToPosition();
    delay(3000);
    virar(stepperNAO, SLEEP_NAO, 60, 1);
    virar(stepperNAO, SLEEP_NAO, 60, -1);
    delay(1000);
  }
  digitalWrite(SLEEP_NAO, LOW);
  Serial.println("LISA tristinha...");
}
