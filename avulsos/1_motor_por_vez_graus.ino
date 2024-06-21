#include <AccelStepper.h>

// Definições de pinos para o motor da cabeça
#define STEP_PIN1 14
#define DIR_PIN1 12

// Definições de pinos para o motor do braço
#define STEP_PIN2 5
#define DIR_PIN2 18

#define STEP_PIN3 19
#define DIR_PIN3 21

#define STEP_PIN4 26
#define DIR_PIN4 33

#define IN41 23
#define IN42 22
#define IN43 4
#define IN44 2

#define LED_BUILTIN 2

// Criação dos objetos stepper para a cabeça e o braço
AccelStepper stepperSIM(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepperNAO(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepperESQ(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);
AccelStepper stepperBASE(AccelStepper::DRIVER, STEP_PIN4, DIR_PIN4);
AccelStepper stepperDIR(AccelStepper::FULL4WIRE, IN41, IN43, IN42, IN44);

// Definição dos passos por graus
const int passosPor90Graus = 113;  // Número de passos para 90 graus
const int passosPor60Graus = 75;   // Número de passos para 60 graus
const int passosPor120Graus = 150; // Número de passos para 120 graus
int ent;

void setup() {
  // Inicia a comunicação serial com 115200 baud rate.
  Serial.begin(115200);
  
  // Configura o pino do LED como saída
  pinMode(LED_BUILTIN, OUTPUT);
  
  stepperSIM.setMaxSpeed(1000);
  stepperSIM.setAcceleration(500);

  stepperNAO.setMaxSpeed(1000);
  stepperNAO.setAcceleration(500);

  stepperESQ.setMaxSpeed(1000);
  stepperESQ.setAcceleration(500);

  stepperDIR.setMaxSpeed(2000);  // Aumente a velocidade máxima
  stepperDIR.setAcceleration(1000);  // Aumente a aceleração

  stepperBASE.setMaxSpeed(1000);
  stepperBASE.setAcceleration(500);

  ent = 0;
}

void positivo() {
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis > 5000) {
    Serial.println("Movendo Stepper SIM por 90 graus");
    stepperSIM.move(passosPor90Graus); // Move 90 graus
    stepperSIM.runToPosition();
    lastMillis = millis();
  }
}

void negativo() {
  // Move para a posição inicial (90 graus)
  Serial.println("Movendo Stepper NAO para a posição inicial de 90 graus");
  stepperNAO.moveTo(passosPor90Graus);
  stepperNAO.runToPosition();

  for (int i = 0; i < 3; i++) {
    // Move de 90 a 120 graus
    Serial.println("Movendo Stepper NAO de 90 a 120 graus");
    stepperNAO.moveTo(passosPor120Graus);
    stepperNAO.runToPosition();

    // Move de 120 a 60 graus
    Serial.println("Movendo Stepper NAO de 120 a 60 graus");
    stepperNAO.moveTo(passosPor60Graus);
    stepperNAO.runToPosition();

    // Move de 60 a 90 graus
    Serial.println("Movendo Stepper NAO de 60 a 90 graus");
    stepperNAO.moveTo(passosPor90Graus);
    stepperNAO.runToPosition();
  }

  // Volta para a posição inicial (90 graus)
  Serial.println("Volta Stepper NAO para a posição inicial de 90 graus");
  stepperNAO.moveTo(passosPor90Graus);
  stepperNAO.runToPosition();
}

void bracoDireito() {
  // Sobe o braço 90 graus (113 passos)
  Serial.println("Movendo Stepper DIR por 90 graus");
  stepperDIR.moveTo(passosPor90Graus);
  stepperDIR.runSpeedToPosition();  // Utiliza runSpeedToPosition para movimentação rápida
  delay(500);  // Pausa de meio segundo no topo

  // Volta o braço para a posição inicial
  Serial.println("Volta Stepper DIR para a posição inicial");
  stepperDIR.moveTo(0);
  stepperDIR.runSpeedToPosition();  // Utiliza runSpeedToPosition para movimentação rápida
}

void bracoEsquerdo() {
  for (int i = 0; i < 3; i++) {
    // Sobe o braço 90 graus (113 passos)
    Serial.println("Movendo Stepper ESQ por 90 graus");
    stepperESQ.moveTo(passosPor90Graus);
    stepperESQ.runToPosition();
    delay(500);  // Pausa de meio segundo no topo

    // Volta o braço para a posição inicial
    Serial.println("Volta Stepper ESQ para a posição inicial");
    stepperESQ.moveTo(0);
    stepperESQ.runToPosition();
  }
}

void base() {
  // Gira a base 90 graus (113 passos)
  Serial.println("Movendo Stepper BASE por 90 graus");
  stepperBASE.moveTo(passosPor90Graus);
  stepperBASE.runToPosition();
  delay(500);  // Pausa de meio segundo no topo

  // Volta a base para a posição inicial
  Serial.println("Volta Stepper BASE para a posição inicial");
  stepperBASE.moveTo(0);
  stepperBASE.runToPosition();
}

void loop() {
  switch (ent) {
    case 0:
      positivo();
      ent = 1;
      break;
    case 1:
      negativo();
      ent = 2;
      break;
    case 2:
      bracoDireito();
      ent = 3;
      break;
    case 3:
      bracoEsquerdo();
      ent = 4;
      break;
    case 4:
      base();
      ent = 0;
      break;
  }
}
