#include <Arduino.h>

#include <AccelStepper.h>


//SIM
#define IN41 5
#define IN42 18
#define IN43 19
#define IN44 21


#define passosPorRotacao 64
#define microsteps 64

// Ajuste aqui a configuração conforme necessário
AccelStepper stepperSIM(8, IN41, IN43, IN42, IN44);

long graus(float graus) {
  return (graus * passosPorRotacao * microsteps) / 360;
}


void setup() {
  stepperSIM.setMaxSpeed(1000);
  stepperSIM.setAcceleration(500);
  stepperSIM.setSpeed(1000);
  stepperSIM.moveTo(graus(360));
}

void loop() {
  if (stepperSIM.distanceToGo()!=0){
    stepperSIM.run();
  }
  else{
    stepperSIM.setSpeed(0);
  }
}
