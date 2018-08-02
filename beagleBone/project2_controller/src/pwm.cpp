// Helper file to use PWM output pins on the Beaglebone Black Rev C
// Written by Ivan Cortes on 12/1/17
// Used by header file "BBB_GPIO.h"

// MUST LOAD DEVICE TREE OBJECT FILE FIRST, enabling all three PWM output pins
// Compatible file to load: BB-PWM-ALL-00A0.dtbo

#include <iostream>
#include "BBB_GPIO.h"
#include <stdio.h>

using namespace std;

// Enable PWM output on a Beaglebone Black REV C pin
// PWM pins are labeled 0 through 2
// Period input is in nanoseconds
int enablePWM(int pwmNumber, int period){

	// Create file handle, variable to store period value, and pointers
	FILE *PWMHandle = NULL;
	char value[10];
	const char *PWMPeriod;
	const char *PWMDutyCycle;
	const char *PWMEnable;

	// Select file locations based on PWM number
	switch(pwmNumber){
		case 0:
			PWMPeriod = "/sys/class/pwm/pwmchip0/pwm0/period";
        		PWMDutyCycle = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle";
        		PWMEnable = "/sys/class/pwm/pwmchip0/pwm0/enable";
			break;
		case 1:
			PWMPeriod = "/sys/class/pwm/pwmchip2/pwm0/period";
               		PWMDutyCycle = "/sys/class/pwm/pwmchip2/pwm0/duty_cycle";
                	PWMEnable = "/sys/class/pwm/pwmchip2/pwm0/enable";
			break;
		case 2:
			PWMPeriod = "/sys/class/pwm/pwmchip4/pwm0/period";
                	PWMDutyCycle = "/sys/class/pwm/pwmchip4/pwm0/duty_cycle";
                	PWMEnable = "/sys/class/pwm/pwmchip4/pwm0/enable";
			break;
	}

        // Set PWM period (in nanoseconds)
        if((PWMHandle=fopen(PWMPeriod, "r+"))!=NULL){
                sprintf(value, "%d", int(period));
                fwrite(value, sizeof(char), sizeof(value), PWMHandle);
                fclose(PWMHandle);
        }

        // Set default PWM duty cycle (zero nanoseconds)
        if((PWMHandle=fopen(PWMDutyCycle, "r+"))!=NULL){
                fwrite("0", sizeof(char), 7, PWMHandle);
                fclose(PWMHandle);
        }

        // Enable PWM output
        if((PWMHandle=fopen(PWMEnable, "r+"))!=NULL){
                fwrite("1", sizeof(char), 7, PWMHandle);
                fclose(PWMHandle);
        }

	return 0;
}

// Disable PWM output on a Beagleboard Black REV C pin
// PWM pins are labeled 0 through 2
int disablePWM(int pwmNumber){

        // Create file handle, variable to store period value, and pointer
        FILE *PWMHandle = NULL;
        char value[10];
	const char *PWMEnable;

        // Select file locations based on PWM number
        switch(pwmNumber){
        	case 0:
                	PWMEnable = "/sys/class/pwm/pwmchip0/pwm0/enable";
			break;
        	case 1:
                	PWMEnable = "/sys/class/pwm/pwmchip2/pwm0/enable";
			break;
		case 2:
                	PWMEnable = "/sys/class/pwm/pwmchip4/pwm0/enable";
			break;
	}

        // Disable PWM output
        if((PWMHandle=fopen(PWMEnable, "r+"))!=NULL){
                fwrite("0", sizeof(char), 7, PWMHandle);
                fclose(PWMHandle);
        }

        return 0;
}

// PWM output for pin on Beagleboard Black REV C at specified duty cycle
// Must enable pin first using enablePWM() function
// Pins are labeled 0 to 2
int writePWM(int pwmNumber, int duty_cycle){

	// Create file handle and variable to store duty cycle value
	FILE *PWMHandle = NULL;
	char value[10];
	const char *PWMDutyCycle;

	// Chose location to write to based on PWM number
	switch(pwmNumber){
        	case 0:
                	PWMDutyCycle = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle";
			break;
		case 1:
                	PWMDutyCycle = "/sys/class/pwm/pwmchip2/pwm0/duty_cycle";
			break;
		case 2:
                	PWMDutyCycle = "/sys/class/pwm/pwmchip4/pwm0/duty_cycle";
			break;
	}

        // Set PWM duty cycle (high time in nanoseconds)
        if((PWMHandle=fopen(PWMDutyCycle, "r+"))!=NULL){
                sprintf(value, "%d", int(duty_cycle));
                fwrite(value, sizeof(char), sizeof(value), PWMHandle);
                fclose(PWMHandle);
        }

        return 0;
}
