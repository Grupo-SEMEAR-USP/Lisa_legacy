#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <AccelStepper.h>

ros::NodeHandle nh;


// NAO
#define STEP_PIN3 22
#define DIR_PIN3 23
#define SLEEP_NAO 33

#define IN41 5
#define IN42 18
#define IN43 19
#define IN44 21

#define passosPorRotacaoSIM 4096
#define microstepsSIM 1

AccelStepper stepperNAO(1, STEP_PIN3, DIR_PIN3);
AccelStepper stepperSIM(8, IN41, IN43, IN42, IN44);

long grausSIM(float graus) {
  return (graus * passosPorRotacaoSIM * microstepsSIM) / 360;
}

float distancia_x = 0;
float distancia_y = 0;
bool seguirNAO = false;
bool seguirSIM = false;

void centerXCallback(const std_msgs::Float32& msg) {
  distancia_x = msg.data;
  
  if (distancia_x > 60) {
    digitalWrite(2, HIGH);  // Acende o LED no pino 2
    digitalWrite(SLEEP_NAO, HIGH);
    stepperNAO.setSpeed(15);
    seguirNAO = true;  // Permite que o motor rode no loop
  } else if (distancia_x < -60) {
    digitalWrite(2, HIGH);  // Acende o LED no pino 2
    digitalWrite(SLEEP_NAO, HIGH);
    stepperNAO.setSpeed(-15); // Velocidade negativa para inverter o sentido
    seguirNAO = true;  // Permite que o motor rode no loop
  } else {
    digitalWrite(2, LOW);   // Apaga o LED no pino 2
    digitalWrite(SLEEP_NAO, LOW);
    seguirNAO = false; // Impede que o motor rode no loop
  }
}

void centerYCallback(const std_msgs::Float32& msg) {
  distancia_y = msg.data;

  if (distancia_y > 50) {
    if (stepperSIM.currentPosition() > -1024) { // -1024 passos correspondem a -90 graus
      stepperSIM.setSpeed(-200);
      seguirSIM = true;  // Permite que o motor rode no loop
    } else {
      stepperSIM.setSpeed(0);
      seguirSIM = false;
    }
  } else if (distancia_y < -50) {
    if (stepperSIM.currentPosition() < 1024) { // 1024 passos correspondem a 90 graus
      stepperSIM.setSpeed(200); // Velocidade negativa para inverter o sentido
      seguirSIM = true;  // Permite que o motor rode no loop
    } else {
      stepperSIM.setSpeed(0);
      seguirSIM = false;
    }
  } else {
    seguirSIM = false; // Impede que o motor rode no loop
  }
}


ros::Subscriber<std_msgs::Float32> sub_x("/face_center_x", &centerXCallback);
ros::Subscriber<std_msgs::Float32> sub_y("/face_center_y", &centerYCallback);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_x);
  nh.subscribe(sub_y);
  
  // Configura os pinos de sono como saída e os define como 1 (desativados)
  pinMode(SLEEP_NAO, OUTPUT);

  digitalWrite(SLEEP_NAO, LOW);

  
  // Configurações iniciais dos motores
  stepperNAO.setMaxSpeed(1000);
  stepperNAO.setAcceleration(200);
  stepperSIM.setMaxSpeed(1000);
  stepperSIM.setAcceleration(500);
  

  pinMode(2, OUTPUT);  // Configura o pino 2 como saída para controlar o LED
}

void loop() {
  nh.spinOnce();
  
  if (seguirNAO) {
    stepperNAO.runSpeed(); // Mantém o motor da cabeça em movimento constante
  }
    if (seguirSIM) {
    stepperSIM.runSpeed(); // Mantém o motor da cabeça em movimento constante
  }
  
  delay(1);
}
