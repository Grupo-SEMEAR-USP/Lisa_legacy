#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

// Define a margem de erro como 5 passos
#define erro 0

// Definições de pinos para o motor da cabeça
//DIREITA
#define STEP_PIN1 12 
#define DIR_PIN1 14
#define SLEEP_PIN1 13

//ESQUERDA
#define STEP_PIN2 4
#define DIR_PIN2 2
#define SLEEP_PIN2 32

//NAO
#define STEP_PIN3 23
#define DIR_PIN3 22
#define SLEEP_PIN3 33

//BASE
#define STEP_PIN4 25
#define DIR_PIN4 27
#define SLEEP_PIN4 26

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

float distancia_x = 0;
bool seguirNAO = false;
bool devedancar = false;
int dancinha = -1; // Variável para armazenar a dança a ser executada

void centerXCallback(const std_msgs::Float32& msg) {
  distancia_x = msg.data;
  
  if (distancia_x > 100) {
    digitalWrite(2, HIGH);  // Acende o LED no pino 2
    digitalWrite(SLEEP_NAO, HIGH);
    stepperNAO.setSpeed(30);
    seguirNAO = true;  // Permite que o motor rode no loop
  } else if (distancia_x < -100) {
    digitalWrite(2, HIGH);  // Acende o LED no pino 2
    digitalWrite(SLEEP_NAO, HIGH);
    stepperNAO.setSpeed(-30); // Velocidade negativa para inverter o sentido
    seguirNAO = true;  // Permite que o motor rode no loop
  } else {
    digitalWrite(2, LOW);   // Apaga o LED no pino 2
    digitalWrite(SLEEP_NAO, LOW);
    seguirNAO = false; // Impede que o motor rode no loop
  }
}

void resultCallback(const std_msgs::String& msg) {
  // Atualiza a variável danceMove de acordo com a mensagem recebida
  if (msg.data == "cumprimentar") {
    dancinha = 1;
    devedancar = true;
  } else if (msg.data == "feliz") {
    dancinha = 2;
    devedancar = true;
  } else if (msg.data == "sapeca") {
    dancinha = 3;
    devedancar = true;
  } else if (msg.data == "bravinha") {
    dancinha = 4;
    devedancar = true;
  } else if (msg.data == "sim") {
    dancinha = 5;
    devedancar = true;
  } else if (msg.data == "vitoria") {
    dancinha = 6;
    devedancar = true;
  } else if (msg.data == "tristinha") {
    dancinha = 7;
    devedancar = true;
  } else {
    dancinha = -1; // Nenhuma dança
    devedancar = false;
  }
}

ros::Subscriber<std_msgs::Float32> sub_x("/face_center_x", &centerXCallback);
ros::Subscriber<std_msgs::String> sub_result("/resultados", &resultCallback);

// Publishers para devedancar e dancinha
std_msgs::Bool msg_devedancar;
std_msgs::Int32 msg_dancinha;
ros::Publisher pub_devedancar("/devedancar", &msg_devedancar);
ros::Publisher pub_dancinha("/dancinha", &msg_dancinha);

void setup() {
  // Inicia a comunicação serial com 115200 baud rate.
  Serial.begin(115200);

  // Configurações dos motores de passo
  configureStepper(stepperDIR, SLEEP_DIREITA, 300, 500);
  configureStepper(stepperESQ, SLEEP_ESQUERDA, 300, 500);
  configureStepper(stepperNAO, SLEEP_NAO, 1000, 500);
  configureStepper(stepperBASE, SLEEP_BASE, 1000, 500);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_x);
  nh.subscribe(sub_result);

  nh.advertise(pub_devedancar);
  nh.advertise(pub_dancinha);

  pinMode(2, OUTPUT);  // Configura o pino 2 como saída para controlar o LED
}

void loop() {
  nh.spinOnce();
  
  if (seguirNAO && !devedancar) {
    stepperNAO.runSpeed(); // Mantém o motor da cabeça em movimento constante
  }

  if (devedancar) {
    // Publica os valores de devedancar e dancinha
    msg_devedancar.data = devedancar;
    pub_devedancar.publish(&msg_devedancar);

    msg_dancinha.data = dancinha;
    pub_dancinha.publish(&msg_dancinha);

    // Executa a dança correspondente com base na variável danceMove
    switch (dancinha) {
      case 1:
        cumprimentar(stepperDIR);
        break;
      case 2:
        feliz(stepperESQ, stepperDIR);
        break;
      case 3:
        sapeca(stepperBASE, stepperESQ, stepperDIR);
        break;
      case 4:
        bravinha(stepperNAO);
        break;
      case 5:
        sim(stepperSIM);
        break;
      case 6:
        vitoria(stepperBASE);
        break;
      case 7:
        tristinha(stepperSIM, stepperNAO);
        break;
      default:
        // Nenhuma dança
        break;
    }
    devedancar = false; // Reseta o devedancar após a dança
  }

  delay(1);
}


// Função para configurar um motor de passo
void configureStepper(AccelStepper &stepper, int sleepPin, float maxSpeed, float acceleration) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, LOW);
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
void cumprimentar(AccelStepper &stepperDIR) {
  virar120(stepperDIR, SLEEP_DIREITA, 1);
  for (int i = 0; i < 4; i++) {
    virar30(stepperDIR, SLEEP_DIREITA, 1);  // Horário
    virar30(stepperDIR, SLEEP_DIREITA, -1); // Anti-horário
  }
  voltarPara0(stepperDIR, SLEEP_DIREITA);
}

// Função para executar movimentos de felicidade com dois motores
void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR) {
  for (int i = 0; i < 4; i++) {
    virar120(stepperESQ, SLEEP_ESQUERDA, 1);
    virar120(stepperDIR, SLEEP_DIREITA, -1);
    voltarPara0(stepperESQ, SLEEP_ESQUERDA);
    voltarPara0(stepperDIR, SLEEP_DIREITA);
  }
}

// Função para executar movimentos de "sapeca"
void sapeca(AccelStepper &stepperNAO, AccelStepper &stepperESQ, AccelStepper &stepperDIR) {
  for (int i = 0; i < 4; i++) {
    virar120(stepperESQ, SLEEP_ESQUERDA, 1);
    virar120(stepperDIR, SLEEP_DIREITA, -1);
    virar60(stepperNAO, SLEEP_NAO, 1);  // Horário
    voltarPara0(stepperNAO, SLEEP_NAO);
    virar60(stepperNAO, SLEEP_NAO, -1); // Anti-horário
    voltarPara0(stepperESQ, SLEEP_ESQUERDA);
    voltarPara0(stepperDIR, SLEEP_DIREITA);
  }
  voltarPara0(stepperNAO, SLEEP_NAO);
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
  for (int i = 0; i < 4; i++) {
    stepperSIM.moveTo(1000);
    stepperSIM.runToPosition();
    delay(1000);
    stepperSIM.moveTo(0);
    stepperSIM.runToPosition();
    delay(1000);
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
