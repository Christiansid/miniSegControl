#include <Arduino.h>
#include "encoder_arduino.h"

typedef struct
{
    int pinA;
    int pinB;
    long pos;
    int del;
	int Aprev;  //1221
	int Bprev;  //1221
} Encoder;
//volatile Encoder Enc[4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0},{0,0,0,0}};
volatile Encoder Enc[5] = {{0,0,0,0,0,0}, {0,0,0,0,0,0}, {0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}};  //1221

// Auxiliary function to handle encoder attachment
static int getIntNum(int pin)
{
    // Returns the interrupt number for a given interrupt pin
    // See http://arduino.cc/it/Reference/AttachInterrupt
    switch(pin) {
        case 2:
            return 0;
        case 3:
            return 1;
        case 21:
            return 2;
        case 20:
            return 3;
        case 19:
            return 4;
        case 18:
            return 5;
		case 6:        // direct access to interrupt 6
            return 6;
        case 7:
            return 7;  // direct access to interrupt 7
        default:
            return -1;
    }
}

// Auxiliary debouncing function
static void debounce(int del)
{
    for (int k = 0; k < del; k++) {
        // can't use delay in the ISR so need to waste some time
        // perfoming operations, this uses roughly 0.1ms on uno
        k = k +0.0 +0.0 -0.0 +3.0 -3.0;
    }
}

// Interrupt Service Routine: change on pin A for Encoder 0
static void irsPinAEn0(void)
{
    // Read pin B right away
    //int drB = digitalRead(Enc[0].pinB);int drA = digitalRead(Enc[0].pinA);
    int PIND_REG=PIND;  // read entire register once
    int drB = bool(PIND_REG&(1<<2));int drA = bool(PIND_REG&(1<<3));  //PD2, PD3 --> 2, 3
	
    // Possibly wait before reading pin A, then read it
    //debounce(Enc[0].del);
    
    // this updates the counter
    if (drA == HIGH)
    {
        // low->high on A?
        if (drB == LOW) {  // check pin B
            Enc[0].pos++;  // going clockwise: increment
        }
        else {
            Enc[0].pos--;  // going counterclockwise: decrement
        }
        
    }
    else {
        // must be high to low on A
        if (drB == HIGH) { // check pin B
            Enc[0].pos++;  // going clockwise: increment
        }
        else {
            Enc[0].pos--;  // going counterclockwise: decrement
        }
    } // end counter update
} // end ISR pin A Encoder 0

// Interrupt Service Routine: change on pin B for Encoder 0
static void isrPinBEn0(void)
{
    // read pin A right away
    //int drA = digitalRead(Enc[0].pinA);int drB = digitalRead(Enc[0].pinB);
    int PIND_REG=PIND;  // read entire register once
    int drA = bool(PIND_REG&(1<<3));int drB = bool(PIND_REG&(1<<2)); // //PD2, PD3 --> 2, 3
    // possibly wait before reading pin B, then read it
    //debounce(Enc[0].del);

    // this updates the counter
    if (drB == HIGH) {   // low->high on B?
        if (drA == HIGH) { // check pin A
            Enc[0].pos++;  // going clockwise: increment
        } else {
            Enc[0].pos--;  // going counterclockwise: decrement
        }
    } else {                       // must be high to low on B
        if (drA == LOW) {  // check pin A
            Enc[0].pos++;  // going clockwise: increment
        } else {
            Enc[0].pos--;  // going counterclockwise: decrement
        }
    } // end counter update
} // end ISR pin B Encoder 0

// ISR routines for PCINT0
// Encoder using PCINT vector will be ENC[4]
/* Interrupt Service Routine: change on pin A & B */
ISR( PCINT0_vect )
{
  /* read pin A & B right away                                   */
    int PINB_REG=PINB;  // read entire register once
	int drB = bool(PINB_REG&(1<<3));int drA = bool(PINB_REG&(1<<4));  // PB5, PB6 --> 11, 12

	
  // Find out which pin changed:
  if(drB==Enc[4].Bprev){
	  // A must have changed
	  Enc[4].Aprev=drA;
	  // this updates the counter if A Changed:
    if (drA == HIGH) {   // low->high on A?
        if (drB == LOW) {  // check pin B
            Enc[4].pos++;  // going clockwise: increment
        } else {
            Enc[4].pos--;  // going counterclockwise: decrement
        }
    }
    else { // must be high to low on A
        if (drB == HIGH) { // check pin B
            Enc[4].pos++;  // going clockwise: increment
        } else {
            Enc[4].pos--;  // going counterclockwise: decrement
        }
    } // end counter update
	
  } else {
	  // B must have changed
	  Enc[4].Bprev=drB;
	  // this updates the counter if B changed:
    if (drB == HIGH) {   // low->high on B?
        if (drA == HIGH) { // check pin A
            Enc[4].pos++;  // going clockwise: increment
        } else {
            Enc[4].pos--;  // going counterclockwise: decrement
        }
    }
    else { // must be high to low on B
        if (drA == LOW) {  // check pin A
            Enc[4].pos++;  // going clockwise: increment
        } else {
            Enc[4].pos--;  // going counterclockwise: decrement
        }
    } // end counter update
	  
  }
	
  /* this updates the counter                                */
  if (drA == HIGH) {   /* low->high on A? */
      
    if (drB == LOW) {  /* check pin B */
  	Enc[4].pos++;  /* going clockwise: increment         */
    } else {
  	Enc[4].pos--;  /* going counterclockwise: decrement  */
    }
    
  } else {                       /* must be high to low on A */
  
    if (drB == HIGH) { /* check pin B */
  	Enc[4].pos++;  /* going clockwise: increment         */
    } else {
  	Enc[4].pos--;  /* going counterclockwise: decrement  */
    }
    
  } /* end counter update                                    */

} /* end ISR pin A Encoder 3     */


// Initialization function called by Encoder System object
extern "C" void enc_init(int enc, int pinA, int pinB)
{
    // enc is the encoder number and it can be 0,1 or 2
    // if other encoder blocks are present in the model
    // up to a maximum of 3, they need to refer to a
    // different encoder number
    
    // store pinA and pinB in global encoder structure Enc
    // they will be needed later by the interrupt routine
    // that will not be able to access s-function parameters
    
    Enc[enc].pinA=pinA;      // set pin A
    Enc[enc].pinB=pinB;      // set pin B
    
    // set encoder pins as inputs
    pinMode(Enc[enc].pinA, INPUT);
    pinMode(Enc[enc].pinB, INPUT);
    
    // turn on pullup resistors
    digitalWrite(Enc[enc].pinA, HIGH);
    digitalWrite(Enc[enc].pinB, HIGH);
    
    // attach interrupts
    switch(enc) {
        case 0:
            attachInterrupt(getIntNum(Enc[0].pinA), irsPinAEn0, CHANGE);
            attachInterrupt(getIntNum(Enc[0].pinB), isrPinBEn0, CHANGE);
            break;
        case 1:
            //attachInterrupt(getIntNum(Enc[1].pinA), irsPinAEn1, CHANGE);
            //attachInterrupt(getIntNum(Enc[1].pinB), isrPinBEn1, CHANGE);
            break;
        case 2:
            //attachInterrupt(getIntNum(Enc[2].pinA), irsPinAEn2, CHANGE);
            //attachInterrupt(getIntNum(Enc[2].pinB), isrPinBEn2, CHANGE);
            break;
		case 3:
			// No Case 3 for uno nano
			// attach interrupts for enc[3]:
			// make analog pin A8 and RX3 (D13) into pin change interrupts:
			//PCICR |= (1 << PCIE2);     // enable PCINT 2
			//PCICR |= (1 << PCIE1);     // enable PCINT 1 
			//PCMSK2 |= (1 << PCINT16);  // Analog 8 as input
			//PCMSK1 |= (1 << PCINT9);  // RX3 D15 as input 
			break;		
		case 4:
			// attach interrupts for enc[4]:
			// ping 11 and 12 into pin change interrupts:
			PCICR |= (1 << PCIE0);     // enable PCINT0  (pins 11 and 12 --> PCINT5, PCINT6 --> PCINT0 vec)
			PCMSK0 |= (1 << PCINT3);  // Pin 11 (uno, nano)
			PCMSK0 |= (1 << PCINT4);  // Pin 12 (uno, nano)
			break;			
    }
}

// Output function called by Encoder System object
extern "C" long enc_output(long enc)
{
    return (long)Enc[enc].pos;
}
// [EOF]

