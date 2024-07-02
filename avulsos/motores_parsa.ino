#include <AccelStepper.h>


// Definições de pinos para o motor da cabeça
//DIREITA
#define STEP_PIN1 12 
#define DIR_PIN1 14
#define SLEEP_PIN1 13

//ESQUERDA
#define STEP_PIN2 2
#define DIR_PIN2 4
#define SLEEP_PIN2 32 //16 //34 35 DONT (1 proibido usar trava serial)

//NAO
#define STEP_PIN3 23
#define DIR_PIN3 21
#define SLEEP_PIN3 33 //17

//BASE
#define STEP_PIN4 25
#define DIR_PIN4 27
#define SLEEP_PIN4 26

//SIM
#define IN41 5
#define IN42 18
#define IN43 19
#define IN44 21


// Criação dos objetos stepper para a cabeça e o braço
AccelStepper stepperDIR(1, STEP_PIN1, DIR_PIN1);
AccelStepper stepperESQ(1, STEP_PIN2, DIR_PIN2);
AccelStepper stepperNAO(1, STEP_PIN3, DIR_PIN3);
AccelStepper stepperBASE(1, STEP_PIN4, DIR_PIN4);
AccelStepper stepperSIM(8, IN41, IN43, IN42, IN44);


void setup() {
  // Inicia a comunicação serial com 115200 baud rate.
  Serial.begin(115200);
  
  // Configura o pino do LED como saída
  
  stepperDIR.setMaxSpeed(300);
  stepperDIR.setAcceleration(100);
  pinMode(SLEEP_PIN1, OUTPUT);
  digitalWrite(SLEEP_PIN1, LOW);

  stepperESQ.setMaxSpeed(300);
  stepperESQ.setAcceleration(100);
  pinMode(SLEEP_PIN2, OUTPUT);
  digitalWrite(SLEEP_PIN2, LOW);

  stepperNAO.setMaxSpeed(1000);
  stepperNAO.setAcceleration(500);
  pinMode(SLEEP_PIN3, OUTPUT);
  digitalWrite(SLEEP_PIN3, LOW);

  stepperBASE.setMaxSpeed(1000);
  stepperBASE.setAcceleration(500);
  pinMode(SLEEP_PIN4, OUTPUT);
  digitalWrite(SLEEP_PIN4, LOW);
}


void loop() {

  digitalWrite(SLEEP_PIN1, HIGH);
  if (stepperDIR.currentPosition() == 0){
    stepperDIR.moveTo(50);
    stepperSIM.setSpeed(500);
  } 
  else if (stepperDIR.currentPosition() == 50){
    stepperDIR.moveTo(0);
    stepperSIM.setSpeed(500);
  }
  stepperDIR.run();

  digitalWrite(SLEEP_PIN2, HIGH);
  if (stepperESQ.currentPosition() == 0){
    stepperESQ.moveTo(50);
    stepperSIM.setSpeed(500);
  } 
  else if (stepperESQ.currentPosition() == 50){
    stepperESQ.moveTo(0);
    stepperSIM.setSpeed(500);
  }
  stepperESQ.run();

  
  stepperSIM.runSpeed();
  
  }
