#include <AccelStepper.h>

#define erro 0  // Define a margem de erro como 5 passos

// Definições de pinos para o motor da cabeça
#define STEP_PIN1 12
#define DIR_PIN1 14
#define SLEEP_PIN1 13

#define STEP_PIN2 2
#define DIR_PIN2 4
#define SLEEP_PIN2 32

#define STEP_PIN3 23
#define DIR_PIN3 21
#define SLEEP_PIN3 33

#define STEP_PIN4 25
#define DIR_PIN4 27
#define SLEEP_PIN4 26

#define IN41 5
#define IN42 18
#define IN43 19
#define IN44 21

#define passosPorRotacao 200
#define microsteps 1

long graus(float graus) {
  return (graus * passosPorRotacao * microsteps) / 360;
}

class Motor {
public:
  Motor(int stepPin, int dirPin, int sleepPin, int interfaceType = 1)
    : stepper(interfaceType, stepPin, dirPin), sleepPin(sleepPin) {
    pinMode(sleepPin, OUTPUT);
    digitalWrite(sleepPin, LOW);
  }

  virtual void configure(float maxSpeed, float acceleration) {
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
  }

  virtual void virar(float graus, int direcao) {
    digitalWrite(sleepPin, HIGH);
    stepper.move(::graus(graus) * direcao);
    while (abs(stepper.distanceToGo()) > erro) {
      stepper.run();
    }
    Serial.print("Motor moved ");
    Serial.print(graus);
    Serial.print(" degrees in direction: ");
    Serial.println(direcao);
  }

  virtual void voltarPara0() {
    digitalWrite(sleepPin, HIGH);
    stepper.moveTo(0);
    while (abs(stepper.distanceToGo()) > erro) {
      stepper.run();
    }
    Serial.println("Motor moved back to 0 degrees");
  }

  // Funções específicas de graus
  void virar15(int direcao) {
    virar(15, direcao);
  }

  void virar30(int direcao) {
    virar(30, direcao);
  }

  void virar45(int direcao) {
    virar(45, direcao);
  }

  void virar60(int direcao) {
    virar(60, direcao);
  }

  void virar120(int direcao) {
    virar(120, direcao);
  }

protected:
  AccelStepper stepper;
  int sleepPin;
};

class MotorSIM : public Motor {
public:
  MotorSIM(int in1, int in2, int in3, int in4)
    : Motor(0, 0, 0, 8), stepperSIM(8, in1, in3, in2, in4) {
    // Não há pino de sleep para este motor
  }

  void configure(float maxSpeed, float acceleration) override {
    stepperSIM.setMaxSpeed(maxSpeed);
    stepperSIM.setAcceleration(acceleration);
  }

  void virar(float graus, int direcao) override {
    digitalWrite(sleepPin, HIGH);
    stepperSIM.move(::graus(graus) * direcao);
    while (abs(stepperSIM.distanceToGo()) > erro) {
      stepperSIM.run();
    }
    Serial.print("MotorSIM moved ");
    Serial.print(graus);
    Serial.print(" degrees in direction: ");
    Serial.println(direcao);
  }

  void voltarPara0() override {
    digitalWrite(sleepPin, HIGH);
    stepperSIM.moveTo(0);
    while (abs(stepperSIM.distanceToGo()) > erro) {
      stepperSIM.run();
    }
    Serial.println("MotorSIM moved back to 0 degrees");
  }

protected:
  AccelStepper stepperSIM;
};

// Declaração das funções cumprimentar e feliz
void cumprimentar(Motor &motor);
void feliz(Motor &motorESQ, Motor &motorDIR);

// Instanciação dos objetos de motor
Motor stepperDIR(STEP_PIN1, DIR_PIN1, SLEEP_PIN1);
Motor stepperESQ(STEP_PIN2, DIR_PIN2, SLEEP_PIN2);
Motor stepperNAO(STEP_PIN3, DIR_PIN3, SLEEP_PIN3);
Motor stepperBASE(STEP_PIN4, DIR_PIN4, SLEEP_PIN4);
MotorSIM stepperSIM(IN41, IN42, IN43, IN44);

void setup() {
  Serial.begin(115200);

  stepperDIR.configure(300, 500);
  stepperESQ.configure(300, 175);
  stepperNAO.configure(1000, 500);
  stepperBASE.configure(1000, 500);
  stepperSIM.configure(1000, 500);  // Configuração específica para o MotorSIM
}

void loop() {
  cumprimentar(stepperDIR);
  // feliz(stepperESQ, stepperDIR);
}

void cumprimentar(Motor &motor) {
  // Move para 60 graus (horário)
  motor.virar120(1);
cumprimentar
  // Variar 30 graus (horário e anti-horário) 4 vezes
  for (int i = 0; i < 4; i++) {
    motor.virar30(1);  // Horário
    motor.virar30(-1); // Anti-horário
  }

  // Voltar para 0 (anti-horário)
  motor.voltarPara0();
}

void feliz(Motor &motorESQ, Motor &motorDIR) {
  for (int i = 0; i < 4; i++) {
    motorESQ.virar60(-1); // Motor esquerdo anti-horário
    motorDIR.virar60(1);  // Motor direito horário
    motorESQ.voltarPara0();
    motorDIR.voltarPara0();
  }
}
