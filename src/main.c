/* -------------------------------------------------------------------------
ec18446 - Project B:

This projects implements the following features:

- A cyclic system is used to implement the project features
- Variable-speed stepping is done by using Channel 0 of the PIT module
according to the load value set in the timer.
- Six moves with different speeds, net period and steps.
- A button is used to start and iterate through the six moves
if the motor is at its start position.
- A button is used to stop the motor on its first press event,
on its second press event the motor returns to its start position.

The libraries math.h and stdlib.h are added in the project to use:
- int abs(int n): Computes the absolute value of an integer number.
- int floorf(float arg): Computes the largest integer value not greater than arg.

The following GPIO pins are used for the motor:
   Motor Cnnctn   Port E Pin
-----------------------------
    IN1           pin 30       (phase A+)
    IN2           pin 29       (phase A-)
    IN3           pin 23       (phase B+)
    IN4           pin 22       (phase B-)
The following GPIO pins are used for the buttons:
	Button	Pin
------------------
	Start		PTD6
	Stop		PTD7
------------------------------------------------------------------------- */

#include <MKL25Z4.H>
#include <stdbool.h>
#include <math.h>		// to use floorf
#include <stdlib.h>	// to use abs
#include "../include/gpio_defs.h"
#include "../include/stepperMotor.h"
#include "../include/pit.h"
#include "../include/SysTick.h"

/*----------------------------------------------------------------------------
  Motor Configuration

 *----------------------------------------------------------------------------*/
motorType mcb ;   // motor control block
MotorId m1 ;      // motor id
int sys_state = STATESTART ;	// current state of the system

void configureMotor() {
    m1 = & mcb ;
    m1->port = PTE ;
    m1->bitAp = MOTOR_IN1 ;
    m1->bitAm = MOTOR_IN2 ;
    m1->bitBp = MOTOR_IN3 ;
    m1->bitBm = MOTOR_IN4 ;

    // Enable clock to port E
    SIM->SCGC5 |=  SIM_SCGC5_PORTE_MASK; /* enable clock for port E */
    
    // Initialise motor data and set to state 1
    initMotor(m1) ; // motor initially stopped, with step 1 powered
}
/*----------------------------------------------------------------------------
  Poll the Start Button

 Detect changes in the switch state.
    isStartPressed and not closed --> new press; 
    ~isStartPressed and closed -> not closed
*----------------------------------------------------------------------------*/
int start_b_state = BUTTONOPEN;
int startPressed = false;
int start_bounceCounter = 0;

void task1PollStartButton(void){
    if (start_bounceCounter > 0) start_bounceCounter -- ;
    
    switch (start_b_state) {
			case BUTTONOPEN:
				if (isStartPressed()) {
					startPressed = true ;  // set start button flag
					start_b_state = BUTTONCLOSED;
				}
				break;
			case BUTTONCLOSED:
				if (!isStartPressed()) {
					start_b_state = BUTTONBOUNCE;
					start_bounceCounter = 50;
         }
				break;
			case BUTTONBOUNCE:
				if (isStartPressed()) {
					start_b_state = BUTTONCLOSED;
				}
				if (start_bounceCounter == 0) {
					start_b_state = BUTTONOPEN;
				}
				break;
    }
}
/*----------------------------------------------------------------------------
  Poll the Stop Button

 Detect changes in the switch state.
    isStopPressed and not closed --> new press; 
    ~isStopPressed and closed -> not closed
*----------------------------------------------------------------------------*/
int stop_b_state = BUTTONOPEN;
int stopPressed = false;
int stop_bounceCounter = 0;

void task2PollStopButton(void){
    if (stop_bounceCounter > 0) stop_bounceCounter --;
    
    switch (stop_b_state) {
			case BUTTONOPEN:
				if (isStopPressed()) {
					stopPressed = true;  // create a 'pressed' event
					stop_b_state = BUTTONCLOSED;
				}
				break;
			case BUTTONCLOSED:
				if (!isStopPressed()) {
					stop_b_state = BUTTONBOUNCE;
					stop_bounceCounter = 50;
				}
				break;
			case BUTTONBOUNCE:
				if (isStopPressed()) {
					stop_b_state = BUTTONCLOSED;
				}
				if (stop_bounceCounter == 0) {
					stop_b_state = BUTTONOPEN;
				}
				break;
    }
}
/*----------------------------------------------------------------------------
	task3 Motor Control
		- the motor is initially stopped
    -   
*----------------------------------------------------------------------------*/
enum moves_steps {
STEPS1 = 64,	//  1 1/3 turns (clockwise)
STEPS2 = 272, //  5 2/3 turns (clockwise)
STEPS3 = 736,	// 15 1/3 turns (anticlockwise)
STEPS4 = 512, // 10 2/3 turns (anticlockwise)
STEPS5 = 960, // 20 turns (anticlokwise)
STEPS6 = 1472}// 30 2/3 turns (clockwise)
m_steps = STEPS1, *pSteps = &m_steps;

enum moves_counts {
COUNT1 = 3276799,	// 20s
COUNT2 = 771011,	// 20s
COUNT3 = 284938,	// 20s
COUNT4 = 204799,	// 10s
COUNT5 = 109226,	// 10s
COUNT6 = 71234}		// 10s
m_counts = COUNT1, *pCounts = &m_counts;	

// Set next move in a loop: move 1...move 6...move 1..etc
void setNextMove(enum moves_steps* currentSteps, enum moves_counts* currentCounts, bool* clockwise){
	switch(*currentSteps){
		case STEPS1:
			*currentSteps = STEPS2;
			*currentCounts = COUNT2;
			*clockwise = true;
		break;
		case STEPS2:
			*currentSteps = STEPS3;
			*currentCounts = COUNT3;
			*clockwise = false;
		break;
		case STEPS3:
			*currentSteps = STEPS4;
			*currentCounts = COUNT4;
			*clockwise = false;
		break;
		case STEPS4:
			*currentSteps = STEPS5;
			*currentCounts = COUNT5;
			*clockwise = false;
		break;
		case STEPS5:
			*currentSteps = STEPS6;
			*currentCounts = COUNT6;
			*clockwise = true;
		break;
		case STEPS6:
			*currentSteps = STEPS1;
			*currentCounts = COUNT1;
			*clockwise = true;
		break;
	}
}

// Calculate the least number of steps needed to return to start position
int32_t calculateReturnSteps (int32_t* stepsSinceStart, bool* direction){
    
	int32_t net_rotations = floorf(abs(*stepsSinceStart)/48);	// number of completed rotations
	int32_t remainder_steps = abs(*stepsSinceStart) - (48 * net_rotations);	
	int32_t stepsToReturn = remainder_steps < 24 ? remainder_steps : 48 - remainder_steps;
  if (*stepsSinceStart > 0)
      *direction = remainder_steps < 24 ? false : true;	// false: anticlockwise, true: clockwise
  else
      *direction = remainder_steps < 24 ? true : false;   // true: clockwise, false: anticlockwise
		
	return stepsToReturn;	
}

bool motorRunning = false ;		    // is motor running 
bool clockwise = true;				// current direction of rotation
bool motorAtStartPosition = true;	// is the motor at its start position
int32_t netSteps = 0;               // the raw net steps moved by stepper motor,+ve/-ve
int returnSteps;					// steps to return back to start position
int remainingSteps;				// current number of steps required to complete

void task3ControlMotor(void) {

	switch (sys_state) {
		case STATESTART:
			if (startPressed) {
				startPressed = false; 							// acknowledge event
        moveSteps(m1, *pSteps, clockwise);	    // move clockwise
				setTimer(0, *pCounts);							// set timer load value
				startTimer(0);											// start timer channel 0
				motorAtStartPosition = false;				// motor has moved 
				remainingSteps = *pSteps;
        sys_state = STATERUNNING;						// motor is moving 
			}
			stopPressed = false;	// reset flag if rogue press events
			break;
		case STATERUNNING:
			// remainingSteps may be negative
			if (stopPressed || (!motorRunning && remainingSteps <= 0)) {
				stopPressed = false ; 						// acknowledge event
				stopMotor(m1) ;							    // stop motor
				netSteps = getSteps(m1);	                // get net steps since start
				returnSteps = calculateReturnSteps(&netSteps, &clockwise);
				if (!returnSteps) motorAtStartPosition = true;
				else remainingSteps = returnSteps;
				sys_state = STATESTOPPED;				// motor is stopped 
			}
			startPressed = false;	// reset flag if rogue press events
		break;
		case STATERETURN:
			if(stopPressed && motorRunning){
				stopPressed = false;	// acknowledge event
				stopTimer(0);					// dont update motor
				motorRunning = false;	// must set manually 
															// timer is stopped
			}
			if(stopPressed && !motorRunning){
				stopPressed = false; // acknowledge event
				startTimer(0);			 // update motor
				// no need to set motorRunning = true
				// timer is counting and will update it
			}
			// remainingSteps may be negative 
			if(!motorRunning && remainingSteps <= 0){
				motorAtStartPosition = true;
				sys_state = STATESTOPPED;
			}
			startPressed = false; // reset flag if rogue press events
		break;
		case STATESTOPPED:
			stopTimer(0);	// stop updating motorRunning to avoid bypassing STATERETURN
			// Return to start position
			if (stopPressed && !motorAtStartPosition) {
				stopPressed = false; 										// acknowledge event
				moveSteps(m1, returnSteps, clockwise);	// move back same number of steps
				setTimer(0, *pCounts);	  							// set timer load value
				startTimer(0);													// start timer channel 0
				sys_state = STATERETURN;								// motor is returning to start pos
			}
			
			if (startPressed && motorAtStartPosition) {
				startPressed = false;					    // acknowledge event
				//no need to call stopMotor(m1) motor is already stopped 
				setNextMove(pSteps, pCounts, &clockwise);	// set next move
				moveSteps(m1,*pSteps, clockwise);  			// set move steps
				setTimer(0, *pCounts);										// set timer load value
				startTimer(0);														// start timer channel 0
				motorAtStartPosition = false;   // motor no longer here
				remainingSteps = *pSteps;
				sys_state = STATERUNNING;									// motor is moving 
			}
			
			// reset flags in case of unwated buttons pressed
			stopPressed = false;	
			startPressed = false;
		break ;
	}
}
/* -------------------------------------
    Timer interrupt handler

    Check each channel to see if caused interrupt
    Write 1 to TIF to reset interrupt flag
   ------------------------------------- */
void PIT_IRQHandler(void) {
	// clear pending interrupts
	NVIC_ClearPendingIRQ(PIT_IRQn);

	if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
		// clear TIF
		PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK ;
			
		// add code here for channel 0 interrupt	
		updateMotor(m1);
		motorRunning = isMoving(m1);
		remainingSteps--;	// update steps left
	}

	if (PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK) {
		// clear TIF
		PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK ;
	
		// add code here for channel 1 interrupt
		// -- end of demo code
	}
}
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/

int main (void) {
    configureGPIOoutput() ;
    configureGPIOinput() ;
		configurePIT(0);
    configureMotor() ;
    Init_SysTick(1000) ; // SysTick every ms
    waitSysTickCounter(10) ; // initialise counter
    
    while (1) {        
			task1PollStartButton() ;
			task2PollStopButton() ;
			task3ControlMotor() ;
			waitSysTickCounter(10) ; // cycle every 10ms
    }
}

