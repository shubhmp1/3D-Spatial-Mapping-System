//*********************************************************
// 2DX3 Final Project - Spatial Mapping System
// File: main.c
// Author: Shubh Patel
// Student Number: 400562382
// MacID: pates316
// Date: April 3, 2026
// Description: Embedded spatial scanner using VL53L1X ToF
//              sensor and 28BYJ-48 stepper motor. Collects
//              32 distance measurements per 360 degree scan
//              and transmits data to MATLAB via UART.
//*********************************************************

#include <stdint.h>
#include <stdio.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCR_MFE 0x00000010

// ---- MODE SELECT ----
// 0 = normal operation (motor + ToF + UART + MATLAB)
// 1 = bus speed test only (square wave on PB5, probe with AD3)
#define TEST_MODE 0

uint16_t dev    = 0x29;  // I2C address of VL53L1X
int      status = 0;


//PA0 ~ UART RX
//PA1 ~ UART TX

//*********************************************************
// I2C Init
// PB2 = SCL, PB3 = SDA
// 100 kHz I2C clock using 18 MHz bus
// TPR = (18,000,000 / (2 x 10 x 100,000)) - 1 = 8 = 0x08
//*********************************************************
void I2C_Init(void){
    SYSCTL_RCGCI2C_R  |= SYSCTL_RCGCI2C_R0;    // enable I2C0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;    // enable Port B clock
    while((SYSCTL_PRGPIO_R & 0x0002) == 0){};   // wait for Port B ready
    GPIO_PORTB_AFSEL_R |= 0x0C;                 // alternate function on PB2 PB3
    GPIO_PORTB_ODR_R   |= 0x08;                 // open drain on PB3 (SDA)
    GPIO_PORTB_DEN_R   |= 0x0C;                 // digital enable PB2 PB3
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200; // I2C function
    I2C0_MCR_R  = I2C_MCR_MFE;                 // enable I2C master
    I2C0_MTPR_R = 0x08;                         // 100 kHz at 18 MHz bus speed
}

//*********************************************************
// Bus Speed Test
// Toggles PB5 at 1kHz to verify 18 MHz bus with AD3
//
// At 18 MHz: 1 cycle = 55.5 ns
// Half period for 1kHz = 500us = 500,000 ns
// Loop count = 500,000 / (55.5 x 4) = ~2252
//
// Only runs when TEST_MODE = 1
// loops forever so you can measure as long as you want
//
// AD3 CONNECTIONS:
// Channel 1+ (orange) -> PB5 on board header
// GND        (black)  -> any GND on board
//
// In Waveforms open Scope, hit Run
// Should read exactly 1 kHz = 18 MHz bus confirmed
//*********************************************************
void BusSpeedTest(void){
    // Port B clock already enabled by I2C_Init
    GPIO_PORTB_DIR_R   |=  0x20;
    GPIO_PORTB_DEN_R   |=  0x20;
    GPIO_PORTB_AFSEL_R &= ~0x20;
    GPIO_PORTB_AMSEL_R &= ~0x20;

    // 1 kHz = 0.5ms per half period
    // SysTick_Wait10ms(1) = 180000 cycles = 10ms
    // so 0.5ms = 180000/20 = 9000 cycles
    while(1){
        GPIO_PORTB_DATA_R |=  0x20;
        SysTick_Wait(9000);             // 0.5ms
        GPIO_PORTB_DATA_R &= ~0x20;
        SysTick_Wait(9000);             // 0.5ms
    }
}

//*********************************************************
// Port F Init
// PF4 = Measurement Status LED (flash each ToF reading)
// PF0 = UART Tx Status LED    (on during transmission)
//*********************************************************
void PortF_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
    GPIO_PORTF_DIR_R   |=  0x11;   // PF4 and PF0 outputs
    GPIO_PORTF_DEN_R   |=  0x11;   // digital enable
    GPIO_PORTF_AFSEL_R &= ~0x11;   // no alternate function
    GPIO_PORTF_AMSEL_R &= ~0x11;   // disable analog
}

//*********************************************************
// Port N Init
// PN0 = Additional Status LED (ON while motor scanning CW,
//                              OFF while rewinding CCW)
//*********************************************************
void PortN_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};
    GPIO_PORTN_DIR_R   |=  0x01;   // PN0 output
    GPIO_PORTN_DEN_R   |=  0x01;   // digital enable
    GPIO_PORTN_AFSEL_R &= ~0x01;   // no alternate function
    GPIO_PORTN_AMSEL_R &= ~0x01;   // disable analog
}

//*********************************************************
// Port M Init
// PM3:0 = stepper motor control outputs
//*********************************************************
void PortM_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};
    GPIO_PORTM_DIR_R   |=  0x0F;   // PM3:0 outputs
    GPIO_PORTM_DEN_R   |=  0x0F;   // digital enable
    GPIO_PORTM_AFSEL_R &= ~0x0F;   // no alternate function
    GPIO_PORTM_AMSEL_R &= ~0x0F;   // disable analog
}

//*********************************************************
// Port J Init
// PJ0 = start/stop scan  (onboard button 1)
// PJ1 = reset graph      (onboard button 2)
// both falling edge triggered interrupts with pull-ups
//*********************************************************
void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};
    GPIO_PORTJ_DIR_R   &= ~0x03;   // PJ0 and PJ1 inputs
    GPIO_PORTJ_DEN_R   |=  0x03;   // digital enable
    GPIO_PORTJ_PUR_R   |=  0x03;   // pull-up resistors (active low buttons)
    GPIO_PORTJ_AFSEL_R &= ~0x03;   // no alternate function
    GPIO_PORTJ_AMSEL_R &= ~0x03;   // disable analog
    GPIO_PORTJ_IS_R    &= ~0x03;   // edge sensitive
    GPIO_PORTJ_IBE_R   &= ~0x03;   // single edge only
    GPIO_PORTJ_IEV_R   &= ~0x03;   // falling edge (button press pulls low)
    GPIO_PORTJ_ICR_R    =  0x03;   // clear any prior flags
    GPIO_PORTJ_IM_R    |=  0x03;   // arm interrupts on PJ0 and PJ1
    NVIC_EN1_R         |=  0x00080000; // enable IRQ 51 (Port J)
}

//*********************************************************
// Global flags
// scanning:    0 = idle, 1 = scan in progress
// stopPressed: set by ISR to abort mid-scan
// resetGraph:  set by ISR to tell MATLAB to clear plot
//*********************************************************
volatile int scanning    = 0;
volatile int stopPressed = 0;
volatile int resetGraph  = 0;

//*********************************************************
// Displacement tracking
// scanIndex increments by 1 each completed scan
// MATLAB multiplies by DISPLACEMENT_MM to get real x (mm)
// Per Deliverable 2 spec: pretend each scan is 10 cm apart
//*********************************************************
#define DISPLACEMENT_MM 100   // 100 mm = 10 cm between scans

int scanIndex = 0;            // number of completed scans so far

//*********************************************************
// Port J ISR
// PJ0 - if idle: start scan, if scanning: stop scan
// PJ1 - signal MATLAB to reset/clear the plot
//*********************************************************
void GPIOJ_IRQHandler(void){
    GPIO_PORTJ_ICR_R = 0x03;       // clear both interrupt flags

    SysTick_Wait10ms(2);            // debounce delay (20ms)

    // PJ0 - start/stop
    if((GPIO_PORTJ_DATA_R & 0x01) == 0){
        if(scanning == 0){
            scanning = 1;           // idle -> start scan
        } else {
            stopPressed = 1;        // scanning -> stop and unwind
        }
    }

    // PJ1 - reset graph
    if((GPIO_PORTJ_DATA_R & 0x02) == 0){
        resetGraph = 1;             // main loop will send RESET to MATLAB
    }
}

//*********************************************************
// clockwise
// moves motor one scan position = 11.25 degrees
// 16 full steps of the 4-phase sequence
// checks stopPressed between each step
// returns 1 if completed, 0 if stopped early
//*********************************************************
int clockwise(void){
    for(int j = 0; j < 16; j++){
        if(stopPressed) return 0;
        GPIO_PORTM_DATA_R = 0x03; SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0x06; SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0x0C; SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0x09; SysTick_Wait10ms(1);
    }
    return 1;
}

//*********************************************************
// counterclockwise
// rewinds motor one scan position back
// same step count as clockwise but reversed sequence
// never checks stopPressed so unwind always completes
// (protects wires from tangling)
//*********************************************************
void counterclockwise(void){
    for(int j = 0; j < 16; j++){
        GPIO_PORTM_DATA_R = 0x09; SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0x0C; SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0x06; SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0x03; SysTick_Wait10ms(1);
    }
}

//*********************************************************
// MAIN
//*********************************************************
int main(void){
    uint16_t Distance;
    uint8_t  dataReady = 0;

    // storage for one full scan (32 positions x 11.25 deg = 360)
    uint16_t scanData[32];
    int      scanCount = 0;

    PLL_Init();
    SysTick_Init();
    I2C_Init();

    if(TEST_MODE == 1){
        // bus speed test mode
        // change TEST_MODE to 1 at top of file to enter this mode
        // probe PB5 with AD3 Channel 1+, GND to board GND
        // open Waveforms -> Scope -> Run
        // should read 1 kHz = 18 MHz bus confirmed
        // runs forever until you power off or reflash
        BusSpeedTest();
    }

    // normal operation - only runs when TEST_MODE = 0
    UART_Init();
    PortF_Init();
    PortN_Init();
    PortM_Init();
    PortJ_Init();

    // init ToF sensor and start ranging
    status = VL53L1X_SensorInit(dev);
    status = VL53L1X_StartRanging(dev);

    while(1){

        // ---- RESET GRAPH ----
        if(resetGraph){
            // PF0 flash 100ms - UART tx happening
            GPIO_PORTF_DATA_R |=  0x01;
            SysTick_Wait10ms(10);        // 10 x 10ms = 100ms
            GPIO_PORTF_DATA_R &= ~0x01;

            UART_printf("RESET\r\n");

            if(scanIndex > 0)
                scanIndex--;

            resetGraph = 0;
        }

        // ---- SCAN ----
        if(scanning){

            stopPressed = 0;        // clear stop flag for fresh scan
            scanCount   = 0;        // reset data buffer
            int positionsDone = 0;  // tracks how far motor moved for unwind

            // ---- FORWARD SCAN (CW) ----
            // PN0 ON - motor spinning CW and scanning
            GPIO_PORTN_DATA_R |= 0x01;

            for(int pos = 0; pos < 32; pos++){
                if(stopPressed) break;

                // wait for ToF sensor data ready
                dataReady = 0;
                while(dataReady == 0){
                    VL53L1X_CheckForDataReady(dev, &dataReady);
                    VL53L1_WaitMs(dev, 5);
                }

                GPIO_PORTM_DATA_R = 0x00;   // stop motor briefly while measuring
                SysTick_Wait10ms(1);

                // get distance measurement via I2C
                status = VL53L1X_GetDistance(dev, &Distance);
                status = VL53L1X_ClearInterrupt(dev);

                // store reading in buffer
                scanData[scanCount++] = Distance;

                // PF4 flash - one flash confirms one ToF measurement taken
                GPIO_PORTF_DATA_R |=  0x10;
                SysTick_Wait10ms(5);
                GPIO_PORTF_DATA_R &= ~0x10;

                // advance motor one position CW
                // if stop pressed mid-move clockwise() returns 0
                if(!clockwise()) break;

                positionsDone++;
            }

            // ---- UNWIND (CCW) ----
            // PN0 OFF - motor now reversing back to home
            GPIO_PORTN_DATA_R &= ~0x01;

            for(int pos = 0; pos < positionsDone; pos++){
                counterclockwise();
            }

            // ---- SEND OR DISCARD ----
            if(stopPressed){
                // scan aborted, throw away all readings
                UART_printf("DISCARD\r\n");

            } else {
                // full 360 completed, transmit all data to MATLAB

                // PF0 ON - UART transmission in progress
                GPIO_PORTF_DATA_R |= 0x01;

                // send each scan_index,angle,distance triple
                // MATLAB converts scan_index -> x displacement:
                //   x_mm = scan_index * DISPLACEMENT_MM
                for(int i = 0; i < scanCount; i++){
                    sprintf(printf_buffer, "%d,%d,%u\r\n",
                            scanIndex,
                            (int)(i * 11.25f),
                            scanData[i]);
                    UART_printf(printf_buffer);
                }

                // signal MATLAB that this scan is complete
                UART_printf("END\r\n");

                SysTick_Wait10ms(10);

                // PF0 OFF - transmission done
                GPIO_PORTF_DATA_R &= ~0x01;

                // advance displacement for next button press
                scanIndex++;
            }

            // ---- CLEANUP ----
            GPIO_PORTM_DATA_R  = 0x00;   // de-energise motor coils
            GPIO_PORTN_DATA_R &= ~0x01;  // PN0 OFF
            GPIO_PORTF_DATA_R &= ~0x11;  // PF4 and PF0 OFF
            stopPressed = 0;
            scanning    = 0;             // ready for next button press
        }
    }
}