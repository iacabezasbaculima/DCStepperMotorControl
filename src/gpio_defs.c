#include <MKL25Z4.H>
#include "../include/gpio_defs.h"
#include <stdbool.h>
/*----------------------------------------------------------------------------
  GPIO Configuration

  Configure the port B pin for the on-board red & green leds as an output
 *----------------------------------------------------------------------------*/
void configureGPIOoutput() {
        // Configuration steps
    //   1. Enable clock to GPIO ports
    //   2. Enable GPIO ports
    //   3. Set GPIO direction to output
    //   4. Ensure LEDs are off

    // Enable clock to ports B 
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK ;
    
    // Make the pin GPIO
    PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);          
    PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);          
    
    // Set ports to outputs
    PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS) ;

    // Turn off the LED
    PTB->PSOR = MASK(RED_LED_POS) | MASK(GREEN_LED_POS) ;
}

/*----------------------------------------------------------------------------
  GPIO Input Configuration

  Initialse a Port D pin as an input, with no interrupt
  Bit number given by BUTTON_POS
 *----------------------------------------------------------------------------*/ 
void configureGPIOinput(void) {
    SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK; /* enable clock for port D */

    /* Select GPIO and enable pull-up resistors and no interrupts */
    PORTD->PCR[START_BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
    
    /* Set port D switch bit to inputs */
    PTD->PDDR &= ~MASK(START_BUTTON_POS);
		
		/* Select GPIO and enable pull-up resistors and no interrupts */
    PORTD->PCR[STOP_BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
    
    /* Set port D switch bit to inputs */
    PTD->PDDR &= ~MASK(STOP_BUTTON_POS);
}
/*----------------------------------------------------------------------------
  ledOn: Set led LED on, assumes port B

  Note use of the clear register (c.f. the data output)
 *----------------------------------------------------------------------------*/    
void ledOn(int pos)
{
       // set led on without changing anything else
       // LED is actve low
         PTB->PCOR |= MASK(pos) ;
}

/*----------------------------------------------------------------------------
  ledOff: Set LED off, assumes port B

  Note use of the clear register (c.f. the data output)
 *----------------------------------------------------------------------------*/
void ledOff(int pos)
{
       // set led off with changing anything else
       // LED is actve low
         PTB->PSOR |= MASK(pos) ;
}

/*----------------------------------------------------------------------------
  isStartPressed: test the switch

  Operating the switch connects the input to ground. A non-zero value
  shows the switch is not pressed.
 *----------------------------------------------------------------------------*/
bool isStartPressed(void) {
    if (PTD->PDIR & MASK(START_BUTTON_POS)) {
            return false ;
    }
    return true ;
}
/*----------------------------------------------------------------------------
  isStopPressed: test the switch

  Operating the switch connects the input to ground. A non-zero value
  shows the switch is not pressed.
 *----------------------------------------------------------------------------*/
bool isStopPressed(void) {
    if (PTD->PDIR & MASK(STOP_BUTTON_POS)) {
            return false ;
    }
    return true ;
}
