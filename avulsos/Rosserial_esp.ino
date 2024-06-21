#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

void contadorCallback(const std_msgs::Int32& msg) {
  int fingerCount = msg.data;
  Serial.print("Contador de dedos: ");
  Serial.println(fingerCount);
  
  if (fingerCount == 2) {
    digitalWrite(2, HIGH);  // Acende o LED no pino 2
  } else {
    digitalWrite(2, LOW);   // Apaga o LED no pino 2
  }
}

ros::Subscriber<std_msgs::Int32> sub("/Contador", &contadorCallback);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  pinMode(2, OUTPUT);  // Configura o pino 2 como saída para controlar o LED
}

void loop() {
  nh.spinOnce();
  delay(1);
}
