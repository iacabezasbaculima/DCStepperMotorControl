#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H

#include <stdbool.h>

#define MASK(x) (1UL << (x))

// Freedom KL25Z LEDs
#define RED_LED_POS (18)    // on port B
#define GREEN_LED_POS (19)  // on port B
#define BLUE_LED_POS (1)    // on port D

// Switches
#define START_BUTTON_POS	(6) // PTD6
#define STOP_BUTTON_POS 	(7)	// PTD7

// Button states
#define BUTTONOPEN (0)
#define BUTTONCLOSED (1)
#define BUTTONBOUNCE (2)

// Outputs for stepper motor, on port E
#define MOTOR_IN1 (30) // phase A+
#define MOTOR_IN2 (29) // phase A-
#define MOTOR_IN3 (23) // phase B+
#define MOTOR_IN4 (22) // phase B-

void configureGPIOinput(void);
void configureGPIOoutput(void);
bool isStartPressed(void);
bool isStopPressed(void);
void ledOn(int pos);
void ledOff(int pos);
#endif
