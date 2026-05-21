#include <xc.h>
#define _XTAL_FREQ 4000000

#pragma config FOSC = XT
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config BOREN = ON
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

// ====== MUX PINS ======
//#define S0 PORTAbits.RA1
//#define S1 PORTAbits.RA2
//#define S2 PORTAbits.RA3
//#define S3 PORTAbits.RA5
//SENSORS: 1 2 3 4 5 : RA5 RA3 RA2 RA1 RA0

// ====== TB6612FNG MOTOR PINS ======
#define AIN1 PORTDbits.RD0
#define AIN2 PORTDbits.RD1
#define BIN1 PORTDbits.RD3
#define BIN2 PORTDbits.RD2

// ====== STATUS LED ======
#define STATUS_LED PORTDbits.RD5

#define BASE 125
#define MID 160
#define HALF 62
#define DOUBLE 200
#define SMALL 24

void System_Init(void) {
    // 1. Configure I/O Pins
    TRISCbits.TRISC1 = 0;       // RC1 (CCP2) Output for PWMB (Right Speed)
    TRISCbits.TRISC2 = 0;       // RC2 (CCP1) Output for PWMA (Left Speed)
    TRISD = 0x00;              // RD0, RD1, RD2, RD3 as Outputs for Direction Logic, RD4, RD5 as LED Output
    PORTD = 0xF0;              // Initialize direction pins LOW, Initialize LED HIGH
    ADCON1 = 0x06;	       // Initialize PORTA pins as digital
    TRISA = 0x2F;	       // Initialize RA0 - RA4 as sensor input
   
    // 2. Configure Timer2 for PWM (Recalculated for 4MHz)
    // Target PWM Frequency: ~1kHz (Optimal for N20 motors)
    // Formula: PWM Freq = Fosc / (4 * Prescaler * (PR2 + 1))
    // 4000000 / (4 * 4 * 250) = 1000 Hz
    PR2 = 249;                  // Set period for 1kHz
    T2CON = 0b00000101;         // Timer2 ON, Prescaler 1:4

    // 3. Configure CCP Modules for PWM Mode
    CCP1CON = 0x0C;             // Left Motor PWM Mode
    CCP2CON = 0x0C;             // Right Motor PWM Mode

    // 4. Initialize Duty Cycle at 0 (Stopped)
    CCPR1L = 0;                 
    CCPR2L = 0;    
}

/*
 * Motor Control Function
 * dir: 1 (Forward), -1 (Reverse), 0 (Stop/Brake)
 * speed: 0 to 249 (Max PWM Duty Cycle based on PR2)
 */
void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed) {
    
    // Set Left Motor (A) Direction
    if (leftDir == 1) {
        AIN1 = 1; AIN2 = 0;     //CW, FWD 
    } else if (leftDir == -1) {
        AIN1 = 0; AIN2 = 1;     //CCW, BWD
    } else {
        AIN1 = 0; AIN2 = 0;     
    }

    // Set Right Motor (B) Direction
    if (rightDir == 1) {
        BIN1 = 1; BIN2 = 0;    //CW, FWD   
    } else if (rightDir == -1) {
        BIN1 = 0; BIN2 = 1;     //CCW, BWD
    } else {
        BIN1 = 0; BIN2 = 0;     
    }

    // Apply Speed (0 to 249)
    CCPR1L = leftSpeed;         
    CCPR2L = rightSpeed;        
}

void main(void) {
    System_Init();
    
    __delay_ms(1000);           // Power-on stabilization delay
   STATUS_LED=1;
    while(1) {
      if((PORTA & 0x21) == 0b00101011){ //move forward
	    // 1. Move Forward at ~50% Speed (125/249)
	    setMotors(1, MID, 1, MID);
	 }
	else if((PORTA & 0x2F) == 0x2F){ 
	    //stop
	    setMotors(1, 70, 1, 70);
	}
      else if((PORTA & 0x2F) == 0b00100011){ 
	    // turn slightly left, slight forward;
	    setMotors(1, HALF, 1, BASE);
	 }
      else if((PORTA & 0x2F) == 0b00101001){ 
	    // turn slightly right, slight forward
	    setMotors(1, BASE, 1, HALF);
	 }
      else if((PORTA & 0x2F) == 0b00100111){  //3 and 2 black on left
	    // turn more left
	    setMotors(1, 31, 1, BASE);
	 }
	  else if((PORTA & 0x2F) == 0b00000111){  //3 and 2 black on left
	    // turn more left
	    setMotors(1, 31, 1, DOUBLE);
	 }
      else if((PORTA & 0x2F) == 0b00101101){ //3 and 2 black on right
	    //turn more right
	    setMotors(1, BASE, 1, 31);
	 }
	  else if((PORTA & 0x2F) == 0b00101100){ //3 and 2 black on right
	    //turn more right
	    setMotors(1, DOUBLE, 1, 31);
	 }
      else if((PORTA & 0x2F) == 0b00101110){ 
	   //turn hard right
	    setMotors(1, DOUBLE, 1, SMALL);
	 }
      else if((PORTA & 0x2F) == 0b00001111){ 
	    //turn hard left
	    setMotors(1, SMALL, 1, DOUBLE);
	 }
      else if((PORTA & 0x2F) == 0b00100000 || (PORTA & 0x2F) == 0b00101000){ //black veer off to right 
	   //turn hard right, right wheel stop
	    setMotors(1, MID, -1, 180);
	 }
      else if((PORTA & 0x2F) == 0b00000001 || (PORTA & 0x2F) == 0b00000011){ //black veer off to left 
	    //turn hard left, left wheel stop
	    setMotors(-1, 180, 1, MID);
	 }
      else if((PORTA & 0x2F) == 0x00 || (PORTA & 0x2F) == 0b00100001){ 
	    //stop
	    setMotors(1, 100, 1, 100);
	 }
    }
}
