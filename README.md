Developed for ARM board FRDM-KL25Z
----------------------

This projects implements the following requirements:
-------------------------------------------------
- A cyclic system is used to implement the project features
- Variable-speed stepping is done by using Channel 0 of the PIT module
according to the load value set in the timer.
- Six moves with different speeds, net period and steps.
- A button is used to start and iterate through six moves
provided the motor is at its start position.
- A button is used to stop the motor if it is moving, else 
it causes the motor to return to its start position.

The libraries math.h and stdlib.h are added in this project for:
- int abs(int n): Computes the absolute value of an integer.
- int floorf(float arg): Computes the largest integer value not greater than arg.

The following GPIO pins are used for the motor:
----------------------------------------------- 
Motor Connection Port E Pin

	IN1           pin 30       (phase A+)
	IN2           pin 29       (phase A-)
	IN3           pin 23       (phase B+)
	IN4           pin 22       (phase B-)
    
The following GPIO pins are used for the buttons:
------------------------------------------------	
	Button		Pin
	
	Start		PTD6
	Stop		PTD7
