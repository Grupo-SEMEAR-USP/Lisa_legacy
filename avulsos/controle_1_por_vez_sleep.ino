#include <AccelStepper.h>

// Definições de pinos para o motor da cabeça
#define STEP_PIN1 12
#define DIR_PIN1 14
#define SLEEP_PIN1 13

// Definições de pinos para o motor do braço
#define STEP_PIN2 2
#define DIR_PIN2 4
#define SLEEP_PIN2 16 //34 35 DONT (1 proibido usar trava serial)

#define STEP_PIN3 22
#define DIR_PIN3 23
#define SLEEP_PIN3 17

#define STEP_PIN4 26
#define DIR_PIN4 33
#define SLEEP_PIN4 25

#define IN41 5
#define IN42 18
#define IN43 19
#define IN44 21

#define LED_BUILTIN 2

// Criação dos objetos stepper para a cabeça e o braço
AccelStepper stepper1(1, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(1, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(1, STEP_PIN3, DIR_PIN3);
AccelStepper stepper4(1, STEP_PIN4, DIR_PIN4);
AccelStepper stepper5(8, IN41, IN43, IN42, IN44);

unsigned long lastMillis = 0;
int currentStepper = 0;

void setup() {
  // Inicia a comunicação serial com 115200 baud rate.
  Serial.begin(115200);
  
  // Configura o pino do LED como saída
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Configura os pinos de sono como saída e os define como 1 (desativados)
  pinMode(SLEEP_PIN1, OUTPUT);
  digitalWrite(SLEEP_PIN1, LOW);
  
  pinMode(SLEEP_PIN2, OUTPUT);
  digitalWrite(SLEEP_PIN2, LOW);
  
  pinMode(SLEEP_PIN3, OUTPUT);
  digitalWrite(SLEEP_PIN3, LOW);
  
  pinMode(SLEEP_PIN4, OUTPUT);
  digitalWrite(SLEEP_PIN4, LOW);
  
  // Configurações iniciais do motor
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper1.setSpeed(500); 

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper2.setSpeed(500);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);
  stepper3.setSpeed(500);

  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(500);
  stepper4.setSpeed(500);

  stepper5.setMaxSpeed(2000);
  stepper5.setAcceleration(500);
  stepper5.setSpeed(1000);

  Serial.println("Setup complete");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= 5000) { // A cada 5 segundos
    lastMillis = currentMillis;
    currentStepper = (currentStepper + 1) % 5; // Alterna entre 0 e 4
    // currentStepper = 1;
    Serial.print("Switching to stepper: ");
    Serial.println(currentStepper + 1);

    // Desativa todos os pinos de sono
    digitalWrite(SLEEP_PIN1, LOW);
    digitalWrite(SLEEP_PIN2, LOW);
    digitalWrite(SLEEP_PIN3, LOW);
    digitalWrite(SLEEP_PIN4, LOW);
    // Nota: o stepper5 não tem pino de sono

    // Move o motor para uma nova posição (0 para simplificar)
    switch (currentStepper) {
      case 0:
        digitalWrite(SLEEP_PIN1, HIGH);
        stepper1.moveTo(0);
        stepper1.setSpeed(500); 
        break;
      case 1:
        digitalWrite(SLEEP_PIN2, HIGH);
        stepper2.moveTo(0);
        stepper2.setSpeed(500); 
        break;
      case 2:
        digitalWrite(SLEEP_PIN3, HIGH);
        stepper3.moveTo(0);
        stepper3.setSpeed(500); 
        break;
      case 3:
        digitalWrite(SLEEP_PIN4, HIGH);
        stepper4.moveTo(0);
        stepper4.setSpeed(500); 
        break;
      case 4:
        stepper5.moveTo(0);
        stepper5.setSpeed(1000); 
        break;
    }
  }

  // Executa o motor atual
  switch (currentStepper) {
    case 0:
      stepper1.runSpeed();
      Serial.print("Stepper 1 Speed: ");
      Serial.println(stepper1.speed());
      break;
    case 1:
      stepper2.runSpeed();
      Serial.print("Stepper 2 Speed: ");
      Serial.println(stepper2.speed());
      break;
    case 2:
      stepper3.runSpeed();
      Serial.print("Stepper 3 Speed: ");
      Serial.println(stepper3.speed());
      break;
    case 3:
      stepper4.runSpeed();
      Serial.print("Stepper 4 Speed: ");
      Serial.println(stepper4.speed());
      break;
    case 4:
      stepper5.runSpeed();
      Serial.print("Stepper 5 Speed: ");
      Serial.println(stepper5.speed());
      break;
  }
}
