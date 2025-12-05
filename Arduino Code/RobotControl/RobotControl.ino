/**
 * Combined Motor Control and Ultrasonic Sensor
 * Controls motors via serial commands (w/a/s/d) and continuously reads ultrasonic distance
 */
#include "MeOrion.h"

// Motor setup
MeDCMotor motor1(PORT_1);
MeDCMotor motor2(PORT_2);
MeDCMotor motor3(M1);
MeDCMotor motor4(M2);

// Ultrasonic sensor setup (PORT_3)
MeUltrasonicSensor ultraSensor(PORT_3);

uint8_t motorSpeed = 100;
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 100; // Read sensor every 100ms
uint8_t object_dist = 400;

void setup()
{
  Serial.begin(9600);
  Serial.println("READY - Motor Control + Ultrasonic Sensor Active");
}

void loop()
{
  // Read ultrasonic sensor at regular intervals
  if (millis() - lastSensorRead >= sensorInterval) {
    object_dist = ultraSensor.distanceCm();
    // Serial.print("Distance: ");
    // Serial.print(object_dist);
    // Serial.println(" cm");
    lastSensorRead = millis();
  }

  // Check for motor commands
  if (Serial.available() > 0) { 
    char incomingByte = Serial.read(); 
    Serial.print("Command received: ");
    Serial.println(incomingByte);

    if(incomingByte == 'a') { // turn left
      motor1.run(motorSpeed);
      motor2.run(motorSpeed);
      motor3.run(motorSpeed);
      motor4.run(motorSpeed);
      delay(1000);
    }
      
    if(incomingByte == 'd') { // turn right
      motor1.run(-motorSpeed);
      motor2.run(-motorSpeed);
      motor3.run(-motorSpeed);
      motor4.run(-motorSpeed);
      delay(1000);
    }

    if(incomingByte == 'w') { // move forward
      if(object_dist<=10) {
        Serial.println("Obj1");
      }else{
        Serial.println("Obj0");
        motor1.run(-motorSpeed);
        motor2.run(motorSpeed);
        motor3.run(-motorSpeed);
        motor4.run(motorSpeed);
        delay(1000);
      }
    }

    if(incomingByte == 's') { // move backward
      motor1.run(motorSpeed);
      motor2.run(-motorSpeed);
      motor3.run(motorSpeed);
      motor4.run(-motorSpeed);
      delay(1000);
    }
    
    // Stop all motors after command
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    delay(100);
  }
}
