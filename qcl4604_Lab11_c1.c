/**************************************************************************************************************************

By:                 Quinn Leydon
Date Created:       November 19, 2020
File name:          qcl4604_Lab11_c1.c
Description:        Main C file for Lab 11. Combination of capacitive touch sensor, timer, and UART functionality.
                    The program starts up and gathers baseline measurements from the touch sensors.
                    Next it indicates that the center touch sensor should be pressed.
                    When this is pressed the program then indicates to the user that the up or down sensor should be pressed.
                        If Up pressed: samplerate will be 100Hz.
                        If Down pressed: samplerate will be 200Hz.
                    Next it indicates user should press left or right sensor.
                        If Left pressed: samples will be displayed on linear scale
                        If Right pressed: samples will be displayed on logarithmic scale (base 2)
                    The ADC then reads 20 samples.
                    All samples are then converted to a linear or logarithmic scale.
                    Samples are then displayed in 0.5 second intervals on the CapTouch LEDs. Each time a new sample is
                    displayed the sample is sent over UART and displayed on a terminal on the PC.
                    At any point during the display loop the center touch sensor can be pressed to reset the program
                    to the start of the while(1) loop (waitForUpDown).

**************************************************************************************************************************/

#include <msp430g2553.h>

//char UART_SW_FLAG;

//int uartRxedChar;

//int UART_samples[20];

//extern void setup(void);
extern void waitForCenter(void);
extern void waitForUpDown(void);
extern void waitForLeftRight(void);
extern void getSamples(void);
extern void convertSamples(void);
extern void displaySamples(void);

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
 //   setup();
    waitForCenter();
    while(1)
    {
        waitForUpDown();
        waitForLeftRight();
        getSamples();
        convertSamples();
        displaySamples();
    }


}



