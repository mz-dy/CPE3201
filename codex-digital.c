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

// ====== SENSOR MASK ======
// SENSORS: 1 2 3 4 5 : RA5 RA3 RA2 RA1 RA0
#define SENSOR_MASK 0x2F

// ====== TB6612FNG MOTOR PINS ======
#define AIN1 PORTDbits.RD0
#define AIN2 PORTDbits.RD1
#define BIN1 PORTDbits.RD3
#define BIN2 PORTDbits.RD2

// ====== STATUS LED ======
#define STATUS_LED PORTDbits.RD5

// ====== SPEED SETTINGS ======
#define BASE 110
#define MID 125
#define HALF 55
#define DOUBLE 155
#define SMALL 20
#define PIVOT_REVERSE_SPEED 70
#define PIVOT_FORWARD_SPEED 145
#define SEARCH_REVERSE_SPEED 50
#define SEARCH_FORWARD_SPEED 110

void System_Init(void) {
    // 1. Configure I/O Pins
    TRISCbits.TRISC1 = 0;       // RC1 (CCP2) Output for PWMB (Right Speed)
    TRISCbits.TRISC2 = 0;       // RC2 (CCP1) Output for PWMA (Left Speed)
    TRISD = 0x00;               // RD0, RD1, RD2, RD3 as Outputs for Direction Logic, RD4, RD5 as LED Output
    PORTD = 0xF0;               // Initialize direction pins LOW, Initialize LED HIGH
    ADCON1 = 0x06;              // Initialize PORTA pins as digital
    TRISA = 0x2F;               // Initialize RA0 - RA4 as sensor input

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
        AIN1 = 1; AIN2 = 0;     // CW, FWD
    } else if (leftDir == -1) {
        AIN1 = 0; AIN2 = 1;     // CCW, BWD
    } else {
        AIN1 = 0; AIN2 = 0;
    }

    // Set Right Motor (B) Direction
    if (rightDir == 1) {
        BIN1 = 1; BIN2 = 0;     // CW, FWD
    } else if (rightDir == -1) {
        BIN1 = 0; BIN2 = 1;     // CCW, BWD
    } else {
        BIN1 = 0; BIN2 = 0;
    }

    // Apply Speed (0 to 249)
    CCPR1L = leftSpeed;
    CCPR2L = rightSpeed;
}

void main(void) {
    signed char lastTurn = 0;    // -1 = left, 1 = right, 0 = centered/unknown

    System_Init();

    __delay_ms(1000);           // Power-on stabilization delay
    STATUS_LED = 1;

    while(1) {
        unsigned char sensor = PORTA & SENSOR_MASK;

        if(sensor == 0b00101011 || sensor == 0b00100001) {
            // Centered on the line. Run slower than before so sudden curves are easier to catch.
            setMotors(1, MID, 1, MID);
            lastTurn = 0;
        }
        else if(sensor == 0x2F) {
            // Keep moving slowly through all-white/all-high readings instead of accelerating.
            setMotors(1, 85, 1, 85);
        }
        else if(sensor == 0b00100011) {
            // Slight left correction.
            setMotors(1, HALF, 1, BASE);
            lastTurn = -1;
        }
        else if(sensor == 0b00101001) {
            // Slight right correction.
            setMotors(1, BASE, 1, HALF);
            lastTurn = 1;
        }
        else if(sensor == 0b00100111) {
            // Strong left correction.
            setMotors(1, 31, 1, BASE);
            lastTurn = -1;
        }
        else if(sensor == 0b00000111) {
            // Very strong left correction.
            setMotors(1, 31, 1, DOUBLE);
            lastTurn = -1;
        }
        else if(sensor == 0b00101101) {
            // Strong right correction.
            setMotors(1, BASE, 1, 31);
            lastTurn = 1;
        }
        else if(sensor == 0b00101100) {
            // Very strong right correction.
            setMotors(1, DOUBLE, 1, 31);
            lastTurn = 1;
        }
        else if(sensor == 0b00001111 || sensor == 0b00000001 || sensor == 0b00000011) {
            // Hard left / line far left. Pivot to shrink the turning radius.
            setMotors(-1, PIVOT_REVERSE_SPEED, 1, PIVOT_FORWARD_SPEED);
            lastTurn = -1;
        }
        else if(sensor == 0b00101110 || sensor == 0b00100000 || sensor == 0b00101000) {
            // Hard right / line far right. Pivot to shrink the turning radius.
            setMotors(1, PIVOT_FORWARD_SPEED, -1, PIVOT_REVERSE_SPEED);
            lastTurn = 1;
        }
        else if ((sensor == 0x00) || (sensor == 0x2F)) {
            // Line lost. Search in the last known direction instead of stopping immediately.
            if(lastTurn < 0) {
                setMotors(-1, SEARCH_REVERSE_SPEED, 1, SEARCH_FORWARD_SPEED);
            } else if(lastTurn > 0) {
                setMotors(1, SEARCH_FORWARD_SPEED, -1, SEARCH_REVERSE_SPEED);
            } else {
                setMotors(0, 0, 0, 0);
            }
        }
    }
}
