/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    DCMotorDriverTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/09/09
 * @brief   Description: this file is sample code for Me DC motor device.
 *
 * Function List:
 *    1. void MeDCMotor::run(int16_t speed)
 *    2. void MeDCMotor::stop(void)
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2015/09/09    1.0.0          rebuild the old lib
 * </pre>
 */
#include "MeOrion.h"

MeDCMotor motor1(PORT_1);

MeDCMotor motor2(PORT_2);

MeDCMotor motor3(M1);

MeDCMotor motor4(M2);

uint8_t motorSpeed = 100;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  if (Serial.available() > 0) { 
    // Read the incoming byte (as an integer ASCII value)
    int incomingByte = Serial.read(); 

    // Print what was read for confirmation
    // Serial.print("I received: ");
    // Serial.println(incomingByte); 
    if(incomingByte=='l'){ // take left
      motor1.run(motorSpeed); /* value: between -255 and 255. */
      motor2.run(motorSpeed); /* value: between -255 and 255. */
      motor3.run(motorSpeed);
      motor4.run(motorSpeed);
      delay(2000);
    }
      
    if(incomingByte=='r'){  // take right
      motor1.run(-motorSpeed);
      motor2.run(-motorSpeed);
      motor3.run(-motorSpeed);
      motor4.run(-motorSpeed);
      delay(2000);
    }

    
    if(incomingByte=='f'){ // take front
      motor1.run(-motorSpeed); /* value: between -255 and 255. */
      motor2.run(motorSpeed); /* value: between -255 and 255. */
      motor3.run(-motorSpeed);
      motor4.run(motorSpeed);
      delay(2000);
    }


    if(incomingByte=='b'){ // take back
      motor1.run(motorSpeed); /* value: between -255 and 255. */
      motor2.run(-motorSpeed); /* value: between -255 and 255. */
      motor3.run(motorSpeed);
      motor4.run(-motorSpeed);
      delay(2000);
    }

    else{ // stop
      motor1.stop();
      motor2.stop();
      motor3.stop();
      motor4.stop();
      delay(100);
    }
  }
}

