#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

// Definições de pinos para o motor da cabeça
#define STEP_PIN1 12
#define DIR_PIN1 14
#define SLEEP_PIN1 13

// Definições de pinos para o motor do braço 1
#define STEP_PIN2 2
#define DIR_PIN2 4
#define SLEEP_PIN2 32

// Definições de pinos para o motor do braço 2
#define STEP_PIN3 22
#define DIR_PIN3 23
#define SLEEP_PIN3 33

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);

float distancia_x = 0;
bool shouldRunStepper1 = false;
bool shouldRunStepper2 = false;
bool shouldRunStepper3 = false;

void centerXCallback(const std_msgs::Float32& msg) {
  distancia_x = msg.data;
  
  if (distancia_x > 100) {
    digitalWrite(2, HIGH);  // Acende o LED no pino 2
    digitalWrite(SLEEP_PIN1, HIGH);
    stepper1.setSpeed(30);
    shouldRunStepper1 = true;  // Permite que o motor rode no loop
  } else if (distancia_x < -100) {
    digitalWrite(2, HIGH);  // Acende o LED no pino 2
    digitalWrite(SLEEP_PIN1, HIGH);
    stepper1.setSpeed(-30); // Velocidade negativa para inverter o sentido
    shouldRunStepper1 = true;  // Permite que o motor rode no loop
  } else {
    digitalWrite(2, LOW);   // Apaga o LED no pino 2
    digitalWrite(SLEEP_PIN1, LOW);
    shouldRunStepper1 = false; // Impede que o motor rode no loop
  }
}

void resultCallback(const std_msgs::String& msg) {
  if (msg.data == "Numero de dedos: 4") {
    digitalWrite(SLEEP_PIN2, HIGH);
    digitalWrite(SLEEP_PIN3, HIGH);
    stepper2.setSpeed(30);
    stepper3.setSpeed(-30);
    shouldRunStepper2 = true;
    shouldRunStepper3 = true;
  } else {
    digitalWrite(SLEEP_PIN2, LOW);
    digitalWrite(SLEEP_PIN3, LOW);
    shouldRunStepper2 = false;
    shouldRunStepper3 = false;
  }
}

ros::Subscriber<std_msgs::Float32> sub_x("/face_center_x", &centerXCallback);
ros::Subscriber<std_msgs::String> sub_result("/resultados", &resultCallback);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_x);
  nh.subscribe(sub_result);
  
  // Configura os pinos de sono como saída e os define como 1 (desativados)
  pinMode(SLEEP_PIN1, OUTPUT);
  pinMode(SLEEP_PIN2, OUTPUT);
  pinMode(SLEEP_PIN3, OUTPUT);
  digitalWrite(SLEEP_PIN1, LOW);
  digitalWrite(SLEEP_PIN2, LOW);
  digitalWrite(SLEEP_PIN3, LOW);
  
  // Configurações iniciais dos motores
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(200);
  stepper1.setSpeed(0); // Começa parado
  
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(200);
  stepper2.setSpeed(0); // Começa parado
  
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(200);
  stepper3.setSpeed(0); // Começa parado

  pinMode(2, OUTPUT);  // Configura o pino 2 como saída para controlar o LED
}

void loop() {
  nh.spinOnce();
  
  if (shouldRunStepper1) {
    stepper1.runSpeed(); // Mantém o motor da cabeça em movimento constante
  }
  
  if (shouldRunStepper2) {
    stepper2.runSpeed(); // Mantém o motor do braço 1 em movimento constante
  }
  
  if (shouldRunStepper3) {
    stepper3.runSpeed(); // Mantém o motor do braço 2 em movimento constante
  }
  
  delay(1);
}
