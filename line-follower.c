//==========TEST ANALOG WITH PID===============
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
#define S0 RA1
#define S1 RA2
#define S2 RA3
#define S3 RA5

// ====== TB6612FNG MOTOR PINS ======
#define AIN1 PORTDbits.RD0
#define AIN2 PORTDbits.RD1
#define BIN1 PORTDbits.RD2
#define BIN2 PORTDbits.RD3

// ====== GLOBALS ======
unsigned int sensor[16];

long position;
int error, previousError = 0;

float Kp = 0.6, Ki = 0.0, Kd = 2.0; //HIGH Kp, faster turn per error ; HIGH Kd, smoother turns. LOW Kd, jerky ; HIGH Ki, large overshoot continuous oscillation
float P, I = 0, D, PID;

int baseSpeed = 140;   // safer for 1kHz PWM (0–249 max)
float voltage;							//TEST ONLY
int whole, frac;						//TEST ONLY

void display(){							//TEST ONLY
   PORTB = whole & 0x0F;					//TEST ONLY
   return;							//TEST ONLY
}

// ====== ADC (AN0 ONLY) ======
unsigned int readADC(){
   int d_value = 0;						//TEST ONLY
    GO_nDONE = 1;
    while(GO_nDONE);
       d_value = (ADRESH << 8) | ADRESL;			//TEST ONLY
   voltage = (float)d_value * (float)50 / (float)1023.0;	//TEST ONLY
      frac = ((int)voltage)%10;					//TEST ONLY
      whole = ((int)voltage/10)%10;				//TEST ONLY
      /* setting the LEDs */
      display(); 						//TEST ONLY
    return (ADRESH << 8) | ADRESL;
}

// ====== MUX SELECT (4-bit) ======
void setChannel(unsigned char ch){
    S0 = ch & 0x01;
    S1 = (ch >> 1) & 0x01;
    S2 = (ch >> 2) & 0x01;
    S3 = (ch >> 3) & 0x01;
}

// ====== READ SENSOR ARRAY ======
void readSensors(){
    for(int i = 0; i < 16; i++){
        setChannel(i);
       
        sensor[i] = readADC();
       __delay_ms(500);						//TEST ONLY
    }
}

// ====== POSITION CALC ======
void computePosition(){
    long sum = 0;
    long weighted = 0;

    for(int i = 0; i < 16; i++){
        weighted += (long)sensor[i] * (i * 1000);
        sum += sensor[i];
    }

    if(sum != 0){
        position = weighted / sum; // 0–15000
    }
}

// ====== NEW MOTOR DRIVER FUNCTION ======
void setMotors(signed char leftDir, unsigned char leftSpeed,
               signed char rightDir, unsigned char rightSpeed) {

    // LEFT MOTOR
    if(leftDir == 1){
        AIN1 = 1; AIN2 = 0;
    }
    else if(leftDir == -1){
        AIN1 = 0; AIN2 = 1;
    }
    else{
        AIN1 = 0; AIN2 = 0;
    }

    // RIGHT MOTOR
    if(rightDir == 1){
        BIN1 = 1; BIN2 = 0;
    }
    else if(rightDir == -1){
        BIN1 = 0; BIN2 = 1;
    }
    else{
        BIN1 = 0; BIN2 = 0;
    }

    // PWM LIMIT (IMPORTANT)
    if(leftSpeed > 249) leftSpeed = 249;
    if(rightSpeed > 249) rightSpeed = 249;

    CCPR1L = leftSpeed;
    CCPR2L = rightSpeed;
}

// ====== PID CONTROL ======
void PID_control(){

    error = (position-7500)/10; //IF HIGH VOLTAGE(HIGH D_VALUE) = BLACK ; BUT IF HIGH VOLTAGE(HIGH D_VALUE) = WHITE then error = (7500 - position) / 10;

    P = error;
    I += error;
    D = error - previousError;

    PID = (Kp * P) + (Kd * D) + (Ki * I);

    previousError = error;

    int leftSpeed  = baseSpeed - (int)PID;
    int rightSpeed = baseSpeed + (int)PID;

    // clamp to PWM range
    if(leftSpeed < 0) leftSpeed = 0;
    if(rightSpeed < 0) rightSpeed = 0;

    if(leftSpeed > 249) leftSpeed = 249;
    if(rightSpeed > 249) rightSpeed = 249;

    // determine direction (always forward for line follow)
    setMotors(1, leftSpeed, 1, rightSpeed);
}

// ====== LINE LOST ======
void checkLineLost(){

    int lost = 1;

    for(int i = 0; i < 16; i++){
        if(sensor[i] > 600){
            lost = 0;
            break;
        }
    }

    if(lost){
        if(previousError > 200){
            setMotors(-1, 160, 1, 160); // turn left
        } else if(previousError < -200){
            setMotors(1, 160, -1, 160); // turn right
        } else {
	    setMotors(1, 160, 1, 160); // go straight
	}
    }
}

// ====== INIT ======
void init(){
    TRISD = 0x00;   // motors
    PORTD = 0x00;
    // ADC
   
    ADCON1 = 0x8E;  // AN0 analog only
    ADCON0 = 0x41;  // ADC ON
   TRISA = 0x01;
     TRISC = 0x00;
   PORTC = 0x00; // enable cathode 7-segment LEDs
   TRISB = 0x00; // set all PORTB as output			//TEST ONLY
   PORTB = 0x00; // all LEDs are off				//TEST ONLY
    // PWM (your setup)
    PR2 = 249;
    T2CON = 0b00000101;

    CCP1CON = 0x0C;
    CCP2CON = 0x0C;

    CCPR1L = 0;
    CCPR2L = 0;
}

// ====== MAIN ======
void main(){

    init();

    while(1){

        readSensors();
        computePosition();
        checkLineLost();
        PID_control();
		/* may need to add
		// Only run PID if we haven't lost the line!
        if (!checkLineLost()) {
            PID_control();
        }
		*/
        __delay_ms(5);
    }
}


//========== TEST MOTOR DRIVER =============

/*
 * Compiler: Microchip XC8
 * Target: PIC16F877A
 * Oscillator: 4MHz Crystal (XT)
 * Description: TB6612FNG Motor Driver Test Sequence
 */

/*#include <xc.h>

// Configuration Bits (Updated for 4MHz XT)
#pragma config FOSC = XT   
#pragma config WDTE = OFF  
#pragma config PWRTE = ON  
#pragma config BOREN = ON  
#pragma config LVP = OFF   
#pragma config CPD = OFF   
#pragma config WRT = OFF   
#pragma config CP = OFF    

#define _XTAL_FREQ 4000000     // 4MHz for __delay_ms()

// Motor Direction Pin Mapping (TB6612FNG to PORTD)
#define AIN1 PORTDbits.RD0      // Left Motor Dir 1
#define AIN2 PORTDbits.RD1      // Left Motor Dir 2
#define BIN1 PORTDbits.RD2      // Right Motor Dir 1
#define BIN2 PORTDbits.RD3      // Right Motor Dir 2

void System_Init(void) {
    // 1. Configure I/O Pins
    TRISCbits.TRISC1 = 0;       // RC1 (CCP2) Output for PWMB (Right Speed)
    TRISCbits.TRISC2 = 0;       // RC2 (CCP1) Output for PWMA (Left Speed)
    TRISD = 0x00;              // RD0, RD1, RD2, RD3 as Outputs for Direction Logic, RD4, RD5 as LED Output
    PORTD = 0xF0;              // Initialize direction pins LOW, Initialize LED HIGH

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

    //TRISA = 0x1F;
    
}*/

/*
 * Motor Control Function
 * dir: 1 (Forward), -1 (Reverse), 0 (Stop/Brake)
 * speed: 0 to 249 (Max PWM Duty Cycle based on PR2)
*/
/*void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed) {
    
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

    while(1) {
        // 1. Move Forward at ~50% Speed (125/249)
        setMotors(1, 125, 1, 125);
        __delay_ms(2000);

        // 2. Stop for 1 second
        setMotors(0, 0, 0, 0);
        __delay_ms(9000);

        // 3. Move Reverse at ~50% Speed
        setMotors(-1, 125, -1, 125);
        __delay_ms(2000);

        // 4. Stop for 1 second
        setMotors(0, 0, 0, 0);
        __delay_ms(9000);

        // 5. Pivot Left in Place at ~80% Speed (200/249)
        setMotors(-1, 200, 1, 200);
        __delay_ms(3000);

        // 6. Stop for 2 seconds before looping
        setMotors(0, 0, 0, 0);
        __delay_ms(9000);
    }
}*/

//============ WITH TEST SENSOR INPUT =============

/*
 * Compiler: Microchip XC8
 * Target: PIC16F877A
 * Oscillator: 4MHz Crystal (XT)
 * Description: TB6612FNG Motor Driver Test Sequence
 */

/*#include <xc.h>

// Configuration Bits (Updated for 4MHz XT)
#pragma config FOSC = XT   
#pragma config WDTE = OFF  
#pragma config PWRTE = ON  
#pragma config BOREN = ON  
#pragma config LVP = OFF   
#pragma config CPD = OFF   
#pragma config WRT = OFF   
#pragma config CP = OFF    

#define _XTAL_FREQ 4000000     // 4MHz for __delay_ms()

// Motor Direction Pin Mapping (TB6612FNG to PORTD)
#define AIN1 PORTDbits.RD0      // Left Motor Dir 1
#define AIN2 PORTDbits.RD1      // Left Motor Dir 2
#define BIN1 PORTDbits.RD2      // Right Motor Dir 1
#define BIN2 PORTDbits.RD3      // Right Motor Dir 2

void System_Init(void) {
    // 1. Configure I/O Pins
    TRISCbits.TRISC1 = 0;       // RC1 (CCP2) Output for PWMB (Right Speed)
    TRISCbits.TRISC2 = 0;       // RC2 (CCP1) Output for PWMA (Left Speed)
    TRISD = 0x00;              // RD0, RD1, RD2, RD3 as Outputs for Direction Logic, RD4, RD5 as LED Output
    PORTD = 0xF0;              // Initialize direction pins LOW, Initialize LED HIGH
    ADCON1 = 0x06;	       // Initialize PORTA pins as digital
    TRISA = 0x1F;	       // Initialize RA0 - RA4 as sensor input
   
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
}*/

/*
 * Motor Control Function
 * dir: 1 (Forward), -1 (Reverse), 0 (Stop/Brake)
 * speed: 0 to 249 (Max PWM Duty Cycle based on PR2)
 */
/*void setMotors(signed char leftDir, unsigned char leftSpeed, signed char rightDir, unsigned char rightSpeed) {
    
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

    while(1) {
      if((PORTA & 0x1F) == 0x11 || (PORTA & 0x1F) == 0b00011011){ //move forward
	    // 1. Move Forward at ~50% Speed (125/249)
	    setMotors(1, 125, 1, 125);
	 }
      else if((PORTA & 0x1F) == 0b00010011){ 
	    // turn slightly left, slight forward;
	    setMotors(1, 62, 1, 125);
	 }
      else if((PORTA & 0x1F) == 0b00011001){ 
	    // turn slightly right, slight forward
	    setMotors(1, 125, 1, 62);
	 }
      else if((PORTA & 0x1F) == 0b00010111 || (PORTA & 0x1F) == 0b00000111 || (PORTA & 0x1F) == 0b00000011){ 
	    // turn more left
	    setMotors(1, 31, 1, 200);
	 }
      else if((PORTA & 0x1F) == 0b00011101 || (PORTA & 0x1F) == 0b00011100 || (PORTA & 0x1F) == 0b00011000){ 
	    //turn more right
	    setMotors(1, 200, 1, 31);
	 }
      else if((PORTA & 0x1F) == 0b00011110){ 
	   //turn hard right
	    setMotors(1, 200, 1, 24);
	 }
      else if((PORTA & 0x1F) == 0b00001111){ 
	    //turn hard left
	    setMotors(1, 24, 1, 200);
	 }
      else if((PORTA & 0x1F) == 0b00010000){ //near veer off to right 
	   //turn hard left, left wheel stop
	    setMotors(0, 0, 1, 125);
	 }
      else if((PORTA & 0x1F) == 0b00000001){ //near veer off to left 
	    //turn hard right, right wheel stop
	    setMotors(1, 125, 0, 0);
	 }
      else if((PORTA & 0x1F) == 0x00 || (PORTA & 0x1F) == 0x00){ 
	    //stop
	    setMotors(0, 0, 0, 0);
	 }
    }
}*/
