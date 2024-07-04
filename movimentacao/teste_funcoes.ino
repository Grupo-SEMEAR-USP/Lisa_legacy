#include <AccelStepper.h>

// Define a margem de erro como 5 passos
#define erro 5  

// Definições de pinos para o motor da cabeça
//DIREITA
#define STEP_PIN1 12 
#define DIR_PIN1 14
#define SLEEP_DIREITA 13

//ESQUERDA
#define STEP_PIN2 22
#define DIR_PIN2  23
#define SLEEP_ESQUERDA 33 //16 //34 35 DONT (1 proibido usar trava serial)

//NAO
#define STEP_PIN3 2
#define DIR_PIN3 4
#define SLEEP_NAO 32 //17

//BASE
#define STEP_PIN4 25
#define DIR_PIN4 27 
#define SLEEP_BASE 26

//SIM
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
void virar120(AccelStepper &stepper, int sleepPin, int direcao);
void virar90(AccelStepper &stepper, int sleepPin, int direcao);
void virar60(AccelStepper &stepper, int sleepPin, int direcao);
void virar45(AccelStepper &stepper, int sleepPin, int direcao);
void virar30(AccelStepper &stepper, int sleepPin, int direcao);
void virar15(AccelStepper &stepper, int sleepPin, int direcao);
void voltarPara0(AccelStepper &stepper, int sleepPin);
void cumprimentar(AccelStepper &stepperDIR);
void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR);
void sapeca(AccelStepper &stepperBASE, AccelStepper &stepperESQ, AccelStepper &stepperDIR);
void bravinha(AccelStepper &stepperNAO);
void sim(AccelStepper &stepperSIM);
void vitoria(AccelStepper &stepperBASE);
void tristinha(AccelStepper &stepperSIM, AccelStepper &stepperNAO);
void bombastic_sideeye(AccelStepper &stepperNAO);

void setup() {
  // Inicia a comunicação serial com 115200 baud rate.
  Serial.begin(115200);

  // Configurações dos motores de passo
  configureStepper(stepperDIR, SLEEP_DIREITA, 300, 700);
  configureStepper(stepperESQ, SLEEP_ESQUERDA, 300, 700);
  configureStepper(stepperNAO, SLEEP_NAO, 300, 500);
  configureStepper(stepperBASE, SLEEP_BASE, 300, 500);
  stepperSIM.setMaxSpeed(1000);
  stepperSIM.setAcceleration(500);
  // feliz(stepperESQ, stepperDIR);
  // digitalWrite(SLEEP_ESQUERDA, LOW);
  // digitalWrite(SLEEP_DIREITA, LOW);
}

void loop() {
  //cumprimentar(stepperESQ);
  sapeca(stepperSIM, stepperESQ, stepperDIR);
  //sim(stepperSIM);
  //feliz(stepperESQ, stepperDIR);
  //digitalWrite(SLEEP_ESQUERDA, LOW);
  //digitalWrite(SLEEP_DIREITA, LOW);
  // stepperSIM.setSpeed(500);
  // stepperSIM.runSpeed(); 
  
}

// Função para configurar um motor de passo
void configureStepper(AccelStepper &stepper, int sleepPin, float maxSpeed, float acceleration) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, LOW);
}
void virar180(AccelStepper &stepper, int sleepPin, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(180) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved 120 steps in direction: " + String(direcao));
}
// Função para mover o motor em 120 graus
void virar120(AccelStepper &stepper, int sleepPin, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(120) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved 120 steps in direction: " + String(direcao));
}

// Funções para mover o motor em diferentes ângulos
void virar90(AccelStepper &stepper, int sleepPin, int direcao) {
  digitalWrite(sleepPin, HIGH);
  stepper.move(graus(90) * direcao);
  while (abs(stepper.distanceToGo()) > erro) {
    stepper.run();
  }
  Serial.println("Motor moved 90 steps in direction: " + String(direcao));
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
  Serial.println("Motor moved 45 steps in direction: " + String(direcao));
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
  Serial.println("Motor moved 15 steps in direction: " + String(direcao));
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
  
  virar120(stepperESQ, SLEEP_ESQUERDA, 1);
  for (int i = 0; i < 2; i++) {
    stepperESQ.setMaxSpeed(500);
    stepperESQ.setAcceleration(1000);
    virar30(stepperESQ, SLEEP_ESQUERDA, 1);  // Horário
    virar30(stepperESQ, SLEEP_ESQUERDA, -1); // Anti-horário
  }
  stepperESQ.setMaxSpeed(300);
  stepperESQ.setAcceleration(700);
  voltarPara0(stepperESQ, SLEEP_ESQUERDA);
  digitalWrite(SLEEP_ESQUERDA, LOW);

}

// Função para executar movimentos de felicidade com dois motores
void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR) {
  for (int i = 0; i < 2; i++) {
    virar120(stepperESQ, SLEEP_ESQUERDA, 1);
    virar120(stepperDIR, SLEEP_DIREITA, -1);
    stepperESQ.setMaxSpeed(300);
    stepperESQ.setAcceleration(1000);
    stepperDIR.setMaxSpeed(300);
    stepperDIR.setAcceleration(1000);
    voltarPara0(stepperESQ, SLEEP_ESQUERDA);
    voltarPara0(stepperDIR, SLEEP_DIREITA);
  }
stepperESQ.setMaxSpeed(300);
stepperESQ.setAcceleration(700);
stepperDIR.setMaxSpeed(300);
stepperDIR.setAcceleration(700);  
digitalWrite(SLEEP_DIREITA, LOW);
digitalWrite(SLEEP_ESQUERDA, LOW);
}



// Função para executar movimentos de "sapeca"
void sapeca(AccelStepper &stepperSIM, AccelStepper &stepperESQ, AccelStepper &stepperDIR ) {
  // stepperSIM.setSpeed(500);
  // stepperSIM.runSpeed(); 
  for (int i = 0; i < 4; i++) {
    virar120(stepperESQ, SLEEP_ESQUERDA, 1);
    virar120(stepperDIR, SLEEP_DIREITA, -1);
    voltarPara0(stepperESQ, SLEEP_ESQUERDA);
    voltarPara0(stepperDIR, SLEEP_DIREITA);
    sim(stepperSIM);
  }

}

// Função para executar movimentos "bravinha"
void bravinha(AccelStepper &stepperNAO) {
  for (int i = 0; i < 4; i++) {
    virar60(stepperNAO, SLEEP_NAO, 1);
    virar60(stepperNAO, SLEEP_NAO, -1);
  }
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
    virar90(stepperBASE, SLEEP_BASE, 1);
    virar90(stepperBASE, SLEEP_BASE, -1);
  }
}

// Função para executar movimentos de "tristinha"
void tristinha(AccelStepper &stepperSIM, AccelStepper &stepperNAO) {
  for (int i = 0; i < 1; i++) {
    stepperSIM.moveTo(1000);
    stepperSIM.runToPosition();
    delay(3000);
    virar60(stepperNAO, SLEEP_NAO, 1);
    virar60(stepperNAO, SLEEP_NAO, -1);
    delay(1000);
  }
}
