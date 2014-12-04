/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER
#define MAX_SPEED 255
#ifdef USE_BASE
   
#if defined(ADAFRUIT_MOTORSHIELD)
	#include <AFMotor.h>
	AF_DCMotor motorI(1);
	AF_DCMotor motorD(2);
	int direccionI=0;
	int direccionD=0;
	void initMotorController(){
		motorI.run(RELEASE);
		motorD.run(RELEASE);
	}

	void setMotorSpeed(int i, int spd){
		if(spd > MAX_SPEED)
			spd = MAX_SPEED;
		else if (spd < -MAX_SPEED)
			spd = -MAX_SPEED;

		if (i == LEFT){
			if (spd < 0){
				direccionI = 2;
				motorI.run(BACKWARD);
				motorI.setSpeed(abs(spd));
			}
			else if (spd > 0){
				direccionI = 1;
				motorI.run(FORWARD);
				motorI.setSpeed(abs(spd));
			}
			else
				motorI.run(RELEASE);
		}else if (i == RIGHT){
			if (spd < 0){
				direccionD = 2;
				motorD.run(BACKWARD);
				motorD.setSpeed(abs(spd));
			}
			else if (spd > 0){
				direccionD = 1;
				motorD.run(FORWARD);
				motorD.setSpeed(abs(spd));
			}
			else
				motorD.run(RELEASE);
		}
		
	}
	void setMotorSpeeds(int leftSpeed, int rightSpeed) {
		Serial.print("m ");
		Serial.print(leftSpeed);
		Serial.print(" ");
		Serial.println(rightSpeed);
		setMotorSpeed(LEFT, leftSpeed);
		setMotorSpeed(RIGHT, rightSpeed);
	}

#else
	#error A motor driver must be selected!
#endif

#endif
#endif