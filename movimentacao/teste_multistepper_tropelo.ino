#include <AccelStepper.h>
#include <MultiStepper.h>

// Definições de pinos para o motor da cabeça
// DIREITA
#define STEP_PIN1 12 
#define DIR_PIN1 14
#define SLEEP_DIREITA 13

// ESQUERDA
#define STEP_PIN2 22
#define DIR_PIN2  23
#define SLEEP_ESQUERDA 33 //16 //34 35 DONT (1 proibido usar trava serial)

// Criação dos objetos stepper para a cabeça e o braço
AccelStepper stepperDIR(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepperESQ(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

// Create an instance of MultiStepper to control multiple motors simultaneously
MultiStepper steppers;

// Definições para conversão de graus em passos
const int passosPorRotacao = 200; // Ajuste conforme o seu motor de passo
const int microsteps = 16; // Ajuste conforme o seu driver de motor de passo

long graus(float graus) {
  return (graus * passosPorRotacao * microsteps) / 360;
}

void setup() {
  // Inicializa os pinos de sleep como saídas
  pinMode(SLEEP_DIREITA, OUTPUT);
  pinMode(SLEEP_ESQUERDA, OUTPUT);
  
  // Ativa os motores
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  
  // Set the maximum speed and acceleration for each motor
  stepperDIR.setMaxSpeed(500);
  stepperDIR.setAcceleration(200);
  stepperESQ.setMaxSpeed(500);
  stepperESQ.setAcceleration(200);

  // Add the motors to the MultiStepper
  steppers.addStepper(stepperDIR);
  steppers.addStepper(stepperESQ);
}

void moveSimultaneously(long pos1, long pos2) {
  // Define an array to hold the target positions for each motor
  long positions[2];

  // Set target positions for each motor
  positions[0] = pos1; // Target position for motor 1
  positions[1] = pos2; // Target position for motor 2

  // Move the motors to the target positions simultaneously
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all motors have reached their target positions
}

void loop() {
  long pos1 = graus(180);
  long pos2 = graus(60);

  // Exemplo de mover os motores simultaneamente
  moveSimultaneously(pos1, pos2);
  moveSimultaneously(0, 0);
}
