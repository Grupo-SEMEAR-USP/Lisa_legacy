#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

// Define a margem de erro como 5 passos
#define erro 0

// Definições de pinos para os motores
#define STEP_PIN1 12 
#define DIR_PIN1 14
#define SLEEP_DIREITA 13

#define STEP_PIN2 2
#define DIR_PIN2 4
#define SLEEP_ESQUERDA 32 

#define STEP_PIN3 22
#define DIR_PIN3 23
#define SLEEP_NAO 33

#define STEP_PIN4 25
#define DIR_PIN4 27 
#define SLEEP_BASE 26

#define IN41 5
#define IN42 18
#define IN43 19
#define IN44 21

#define passosPorRotacao 200
#define microsteps 1

#define passosPorRotacaoSIM 4096
#define microstepsSIM 1

long graus(float graus) {
  return (graus * passosPorRotacao * microsteps) / 360;
}

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
void resetar_motores();
void cumprimentar(AccelStepper &stepperESQ);
void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR);
void sapeca(AccelStepper &stepperSIM, AccelStepper &stepperNAO, AccelStepper &stepperESQ, AccelStepper &stepperDIR);
void bravinha(AccelStepper &stepperNAO);
void sim(AccelStepper &stepperSIM);
void vitoria(AccelStepper &stepperBASE, AccelStepper &stepperESQ);
void tristinha(AccelStepper &stepperSIM);
void confusa(AccelStepper &stepperNAO, AccelStepper &stepperSIM);
void bombastic(AccelStepper &stepperNAO);
void amor(AccelStepper &stepperNAO, AccelStepper &stepperBASE, AccelStepper &stepperESQ, AccelStepper &stepperDIR);
void dancinha_festa(AccelStepper &stepperESQ, AccelStepper &stepperDIR, AccelStepper &stepperBASE);
void assustada(AccelStepper &stepperESQ, AccelStepper &stepperDIR, AccelStepper &stepperNAO);

float distancia_x = 0;
String resultado;
bool seguirNAO = false;
bool devedancar = false;
int dancinha = -1;

void centerXCallback(const std_msgs::Float32& msg) {
  distancia_x = msg.data;
  
  if (distancia_x > 50) {
    digitalWrite(SLEEP_NAO, HIGH);
    digitalWrite(2, HIGH);
    stepperNAO.setMaxSpeed(1000);
    stepperNAO.setAcceleration(200);
    stepperNAO.setSpeed(30);
    seguirNAO = true;
  } else if (distancia_x < -50) {
    digitalWrite(SLEEP_NAO, HIGH);
    digitalWrite(2, HIGH);
    stepperNAO.setMaxSpeed(1000);
    stepperNAO.setAcceleration(200);
    stepperNAO.setSpeed(-30);
    seguirNAO = true;
  } else {
    digitalWrite(SLEEP_NAO, LOW);
    digitalWrite(2, LOW);
    seguirNAO = false;
  }
}

void resultCallback(const std_msgs::String& msg) {
  resultado = msg.data;
  if (resultado == "Gesto reconhecido: Gesto Open_Palm reconhecido 5 vezes seguidas") {
    dancinha = 1;
    devedancar = true;
  } else if (resultado == "Gesto reconhecido: Gesto Thumb_Up reconhecido 5 vezes seguidas") {
    dancinha = 2;
    devedancar = true;
  } else if (resultado == "Gesto reconhecido: Gesto Pointing_Up reconhecido 5 vezes seguidas") {
    dancinha = 3;
    devedancar = true;
  } else if (resultado == "Gesto reconhecido: Gesto Closed_Fist reconhecido 5 vezes seguidas") {
    dancinha = 4;
    devedancar = true;
  } else if (resultado == "ativando o modo susto") {
    dancinha = 5;
    devedancar = true;
  } else if (resultado == "Gesto reconhecido: Gesto Victory reconhecido 5 vezes seguidas") {
    dancinha = 6;
    devedancar = true;
  } else if (resultado == "Gesto reconhecido: Gesto Thumb_Down reconhecido 5 vezes seguidas") {
    dancinha = 7;
    devedancar = true;
  } else if (resultado == "ativando o modo amor") {
    dancinha = 8;
    devedancar = true;
  } else if (resultado == "ativando o modo festa") {
    dancinha = 9;
    devedancar = true;
  } else {
    dancinha = -1;
    devedancar = false;
  }
}

ros::Subscriber<std_msgs::Float32> sub_x("/face_center_x", &centerXCallback);
ros::Subscriber<std_msgs::String> sub_result("/resultados", &resultCallback);

std_msgs::Bool msg_devedancar;
std_msgs::Int32 msg_dancinha;
ros::Publisher pub_devedancar("/devedancar", &msg_devedancar);
ros::Publisher pub_dancinha("/dancinha", &msg_dancinha);

void setup() {
  Serial.begin(115200);

  configureStepper(stepperDIR, SLEEP_DIREITA, 300, 700);
  configureStepper(stepperESQ, SLEEP_ESQUERDA, 300, 700);
  configureStepper(stepperNAO, SLEEP_NAO, 300, 700);
  configureStepper(stepperBASE, SLEEP_BASE, 100, 300);
  configureStepper(stepperSIM, 0, 1000, 500);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_x);
  nh.subscribe(sub_result);
  nh.advertise(pub_devedancar);
  nh.advertise(pub_dancinha);

  pinMode(2, OUTPUT);
}

void loop() {
  nh.spinOnce();

  if (seguirNAO) {
    stepperNAO.runSpeed();
  }

  if (devedancar) {
    resetar_motores();
    msg_devedancar.data = devedancar;
    pub_devedancar.publish(&msg_devedancar);

    msg_dancinha.data = dancinha;
    pub_dancinha.publish(&msg_dancinha);

    switch (dancinha) {
      case 1:
        bombastic(stepperNAO);
        break;
      case 2:
        feliz(stepperESQ, stepperDIR);
        break;
      case 3:
        sapeca(stepperSIM, stepperNAO, stepperESQ, stepperDIR);
        break;
      case 4:
        bravinha(stepperNAO);
        break;
      case 5:
        assustada(stepperESQ, stepperDIR, stepperNAO);
        break;
      case 6:
        vitoria(stepperBASE, stepperESQ);
        break;
      case 7:
        tristinha(stepperSIM);
        break;
      case 8:
        amor(stepperNAO, stepperBASE, stepperESQ, stepperDIR);
        break;
      case 9:
        dancinha_festa(stepperESQ, stepperDIR, stepperBASE);
        break;
      default:
        break;
    }
    devedancar = false;
  }

  delay(1);
}

void configureStepper(AccelStepper &stepper, int sleepPin, float maxSpeed, float acceleration) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  if (sleepPin != 0) {
    pinMode(sleepPin, OUTPUT);
    digitalWrite(sleepPin, LOW);
  }
}

void resetar_motores(){
  voltarPara0(stepperESQ, SLEEP_ESQUERDA);
  voltarPara0(stepperDIR, SLEEP_DIREITA);
  voltarPara0(stepperBASE, SLEEP_BASE);
  voltarPara0(stepperNAO, SLEEP_NAO);
  stepperSIM.moveTo(0);
  stepperSIM.runToPosition();
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
  virar(stepperESQ, SLEEP_ESQUERDA, 120, -1);
  for (int i = 0; i < 2; i++) {
    stepperESQ.setMaxSpeed(500);
    stepperESQ.setAcceleration(1000);
    virar(stepperESQ, SLEEP_ESQUERDA, 30, -1);
    virar(stepperESQ, SLEEP_ESQUERDA, 30, 1);
  }
  stepperESQ.setMaxSpeed(300);
  stepperESQ.setAcceleration(700);
  voltarPara0(stepperESQ, SLEEP_ESQUERDA);
  Serial.println("Cumprimentando...");
}

void feliz(AccelStepper &stepperESQ, AccelStepper &stepperDIR) {
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  digitalWrite(SLEEP_NAO, HIGH);
  for (int i = 0; i < 3; i++) {
    stepperESQ.setMaxSpeed(100);
    stepperESQ.setAcceleration(300);
    stepperDIR.setMaxSpeed(100);
    stepperDIR.setAcceleration(300);

    stepperESQ.moveTo(-graus(120));
    stepperDIR.moveTo(-graus(60));
    stepperNAO.moveTo(-graus(60));

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperNAO.run();
    }

    stepperESQ.moveTo(graus(60));
    stepperDIR.moveTo(graus(120));
    stepperNAO.moveTo(graus(60));

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperNAO.run();
    }
  }
  stepperESQ.moveTo(graus(0));
  stepperDIR.moveTo(graus(0));
  stepperNAO.moveTo(graus(0));

  while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
    stepperESQ.run();
    stepperDIR.run();
    stepperNAO.run();
  }

  stepperESQ.setMaxSpeed(300);
  stepperESQ.setAcceleration(500);
  stepperDIR.setMaxSpeed(300);
  stepperDIR.setAcceleration(500);  
  digitalWrite(SLEEP_DIREITA, LOW);
  digitalWrite(SLEEP_ESQUERDA, LOW);
  digitalWrite(SLEEP_NAO, LOW);
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
    stepperESQ.moveTo(-graus(60));
    stepperDIR.moveTo(graus(120));
    stepperSIM.moveTo(grausSIM(60));
    stepperNAO.moveTo(graus(60));

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperSIM.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperSIM.run();
      stepperNAO.run();
    }

    stepperESQ.moveTo(0);
    stepperDIR.moveTo(0);
    stepperSIM.moveTo(0);
    stepperNAO.moveTo(0);

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperSIM.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperSIM.run();
      stepperNAO.run();
    }
  }
  
  digitalWrite(SLEEP_DIREITA, LOW);
  digitalWrite(SLEEP_ESQUERDA, LOW);
  digitalWrite(SLEEP_NAO, LOW);
  Serial.println("LISA sapeca...");
}

void bravinha(AccelStepper &stepperNAO) {
  stepperNAO.setMaxSpeed(100);
  stepperNAO.setAcceleration(200);
  for (int i = 0; i < 2; i++) {
    if (i % 2 == 0) {
      virar(stepperNAO, SLEEP_NAO, 60, 1);
      virar(stepperNAO, SLEEP_NAO, 60, -1);
      virar(stepperNAO, SLEEP_NAO, 60, -1);
      virar(stepperNAO, SLEEP_NAO, 60, 1);
    } else {
      virar(stepperNAO, SLEEP_NAO, 60, -1);
      virar(stepperNAO, SLEEP_NAO, 60, 1);
      virar(stepperNAO, SLEEP_NAO, 60, 1);
      virar(stepperNAO, SLEEP_NAO, 60, -1);
    }
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

  stepperSIM.setSpeed(500);

  for (int i = 0; i < 2; i++) {
    stepperNAO.moveTo(graus(60));
    while (stepperNAO.distanceToGo() != 0) {
      stepperNAO.run();
      stepperSIM.runSpeed();
    }

    stepperNAO.moveTo(0);
    while (stepperNAO.distanceToGo() != 0) {
      stepperNAO.run();
      stepperSIM.runSpeed();
    }
  }

  digitalWrite(SLEEP_NAO, LOW);
}

void vitoria(AccelStepper &stepperBASE, AccelStepper &stepperESQ) {
  virar(stepperESQ, SLEEP_ESQUERDA, 120, -1);
  stepperESQ.setMaxSpeed(800);
  stepperESQ.setAcceleration(1000);

  virar(stepperESQ, SLEEP_ESQUERDA, 30, -1);
  virar(stepperBASE, SLEEP_BASE, 60, -1);
  virar(stepperESQ, SLEEP_ESQUERDA, 30, 1);
  virar(stepperBASE, SLEEP_BASE, 120, 1);

  virar(stepperESQ, SLEEP_ESQUERDA, 30, -1);
  virar(stepperBASE, SLEEP_BASE, 120, -1);
  virar(stepperESQ, SLEEP_ESQUERDA, 30, 1);
  virar(stepperBASE, SLEEP_BASE, 120, 1);

  stepperESQ.setMaxSpeed(300);
  stepperESQ.setAcceleration(700);
  voltarPara0(stepperESQ, SLEEP_ESQUERDA);
  voltarPara0(stepperBASE, SLEEP_BASE);
  digitalWrite(SLEEP_BASE, LOW);

  Serial.println("LISA vitoriosa...");
}

void tristinha(AccelStepper &stepperSIM) {
  for (int i = 0; i < 2; i++) {
    stepperSIM.setMaxSpeed(500);
    stepperSIM.setAcceleration(1000);

    stepperSIM.moveTo(-grausSIM(90));
    stepperSIM.runToPosition();
    delay(100);
    stepperSIM.moveTo(0);
    stepperSIM.runToPosition();
    delay(100);
  }
}

void bombastic(AccelStepper &stepperNAO){
  stepperNAO.setMaxSpeed(80);
  stepperNAO.setAcceleration(500);
  delay(400);
  for(int i=0; i<2; i++){
    virar(stepperNAO, SLEEP_NAO, 60, 1);
    static unsigned long lastMillis = 0;
    while(millis() - lastMillis < 100) {
       lastMillis = millis();
    }
    delay(1000);
    virar(stepperNAO, SLEEP_NAO, -60, 1); 
    delay(400);
  }
  voltarPara0(stepperNAO, SLEEP_NAO);
  Serial.println("LISA bombastic side eye...");
}

void amor(AccelStepper &stepperNAO, AccelStepper &stepperBASE, AccelStepper &stepperESQ, AccelStepper &stepperDIR){
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  digitalWrite(SLEEP_NAO, HIGH);
  digitalWrite(SLEEP_BASE, HIGH);

  stepperNAO.setMaxSpeed(100);
  stepperNAO.setAcceleration(500);
  stepperESQ.setMaxSpeed(200);
  stepperESQ.setAcceleration(500);
  stepperDIR.setMaxSpeed(200);
  stepperDIR.setAcceleration(300);
  stepperBASE.setMaxSpeed(100);
  stepperBASE.setAcceleration(300);

  for (int i = 0; i < 2; i++) {
    stepperESQ.moveTo(-graus(90));
    stepperDIR.moveTo(-graus(-90));
    stepperNAO.moveTo(graus(-30));
    stepperBASE.moveTo(graus(-30));

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0 || stepperBASE.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperNAO.run();
      stepperBASE.run();
    }

    delay(500);
    stepperNAO.moveTo(graus(30));
    stepperBASE.moveTo(graus(30));

    while (stepperBASE.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
      stepperBASE.run();
      stepperNAO.run();
    }
    delay(500);

    stepperESQ.moveTo(graus(0));
    stepperDIR.moveTo(graus(0));
    stepperNAO.moveTo(graus(0));
    stepperBASE.moveTo(graus(0));

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0 || stepperBASE.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperNAO.run();
      stepperBASE.run();
    }
    delay(500);
  }
  
  digitalWrite(SLEEP_DIREITA, LOW);
  digitalWrite(SLEEP_ESQUERDA, LOW);
  digitalWrite(SLEEP_NAO, LOW);
  digitalWrite(SLEEP_BASE, LOW);
}

void dancinha_festa(AccelStepper &stepperESQ, AccelStepper &stepperDIR, AccelStepper &stepperBASE){
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  digitalWrite(SLEEP_BASE, HIGH);

  stepperESQ.setMaxSpeed(200);
  stepperESQ.setAcceleration(500);
  stepperDIR.setMaxSpeed(200);
  stepperDIR.setAcceleration(300);
  stepperBASE.setMaxSpeed(100);
  stepperBASE.setAcceleration(500);

  for (int i = 0; i < 2; i++) {
    if (i % 2 == 0) {
      stepperESQ.moveTo(-graus(60));
      stepperDIR.moveTo(-graus(60));
      stepperBASE.moveTo(graus(60));

      while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperBASE.distanceToGo() != 0) {
        stepperESQ.run();
        stepperDIR.run();
        stepperBASE.run();
      }

      stepperESQ.moveTo(-graus(30));
      stepperDIR.moveTo(-graus(-30));
      stepperBASE.moveTo(-graus(30));

      while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperBASE.distanceToGo() != 0) {
        stepperESQ.run();
        stepperDIR.run();
        stepperBASE.run();
      }

      stepperESQ.moveTo(graus(0));
      stepperDIR.moveTo(graus(0));
      stepperBASE.moveTo(graus(0));

      while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperBASE.distanceToGo() != 0) {
        stepperESQ.run();
        stepperDIR.run();
        stepperBASE.run();
      }
    } else {
      stepperESQ.moveTo(graus(60));
      stepperDIR.moveTo(graus(60));
      stepperBASE.moveTo(-graus(60));

      while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperBASE.distanceToGo() != 0) {
        stepperESQ.run();
        stepperDIR.run();
        stepperBASE.run();
      }

      stepperESQ.moveTo(graus(30));
      stepperDIR.moveTo(graus(-30));
      stepperBASE.moveTo(graus(30));

      while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperBASE.distanceToGo() != 0) {
        stepperESQ.run();
        stepperDIR.run();
        stepperBASE.run();
      }

      stepperESQ.moveTo(graus(0));
      stepperDIR.moveTo(graus(0));
      stepperBASE.moveTo(graus(0));

      while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperBASE.distanceToGo() != 0) {
        stepperESQ.run();
        stepperDIR.run();
        stepperBASE.run();
      }
    }
  }

  digitalWrite(SLEEP_BASE, LOW);
  digitalWrite(SLEEP_ESQUERDA, LOW);
  digitalWrite(SLEEP_DIREITA, LOW);
}

void assustada(AccelStepper &stepperESQ, AccelStepper &stepperDIR, AccelStepper &stepperNAO){
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  digitalWrite(SLEEP_NAO, HIGH);

  stepperESQ.setMaxSpeed(100);
  stepperESQ.setAcceleration(500);
  stepperDIR.setMaxSpeed(100);
  stepperDIR.setAcceleration(300);
  stepperNAO.setMaxSpeed(100);
  stepperNAO.setAcceleration(300);

  for (int i = 0; i < 2; i++) {
    stepperESQ.moveTo(-graus(120));
    stepperDIR.moveTo(graus(120));
    stepperNAO.moveTo(graus(20));

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperNAO.run();
    }
    delay(100);

    stepperNAO.moveTo(-graus(20));
    while (stepperNAO.distanceToGo() != 0) {
      stepperNAO.run();
    }

    delay(200);

    stepperESQ.moveTo(-graus(0));
    stepperDIR.moveTo(graus(0));
    stepperNAO.moveTo(graus(0));

    while (stepperESQ.distanceToGo() != 0 || stepperDIR.distanceToGo() != 0 || stepperNAO.distanceToGo() != 0) {
      stepperESQ.run();
      stepperDIR.run();
      stepperNAO.run();
    }
  }
  digitalWrite(SLEEP_DIREITA, LOW);
  digitalWrite(SLEEP_ESQUERDA, LOW);
  digitalWrite(SLEEP_NAO, LOW);
}
