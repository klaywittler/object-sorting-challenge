// Helper file to use digital IO pins on the Beaglebone Black Rev C
// Written by Ivan Cortes on 12/6/17
// Used by header file "BBB_GPIO.h"

#include <iostream>
#include "BBB_GPIO.h"
#include <stdio.h>
#include <string.h>

using namespace std;

// Eable a digital IO pin
// Enter a cmpatible GPIO pin number
int digitalEnable(int pinNumber){
	// Create file handle and variables to store values
        FILE *digitalHandle = NULL;
        char setValue[4], GPIOString[4];
        const char *GPIOExport = "/sys/class/gpio/export";
	sprintf(GPIOString, "%d", pinNumber);

	// Export pin
        if((digitalHandle=fopen(GPIOExport, "r+"))!=NULL){
                strcpy(setValue, GPIOString);
                fwrite(&setValue, sizeof(char), 3, digitalHandle);
                fclose(digitalHandle);
        }

	return 0;
}

// Digital pin output
// Sepecify a GPIO "pinNumber" and a "value" (1 for high, 0 for low)
int digitalWrite(int pinNumber, int value){

	// Create file handle and variables to store values
	FILE *digitalHandle = NULL;
	char setValue[4], GPIOString[4], GPIOValue[64], GPIODirection[64];
	sprintf(GPIOString, "%d", value);
	sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", pinNumber);
	sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", pinNumber);

	// Write direction
	if((digitalHandle=fopen(GPIODirection, "r+"))!=NULL){
                strcpy(setValue, "out");
		fwrite(&setValue, sizeof(char), 3, digitalHandle);
                fclose(digitalHandle);
        }

        // Write value
        if((digitalHandle=fopen(GPIOValue, "r+"))!=NULL){
        	strcpy(setValue, GPIOString);
	        fwrite(&setValue, sizeof(char), 3, digitalHandle);
                fclose(digitalHandle);
        }

        return 0;
}

// Digital pin read
// Specify a GPIO "pinNumber", returns the pin state (1 for high, 0 for low)
int digitalRead(int pinNumber){

        // Create file handle and variables to store values
        FILE *digitalHandle = NULL;
	char readVal;
        char setValue[4], GPIOValue[64], GPIODirection[64];
        sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", pinNumber);
        sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", pinNumber);

        // Write direction
        if((digitalHandle=fopen(GPIODirection, "r+"))!=NULL){
                strcpy(setValue, "in");
                fwrite(&setValue, sizeof(char), 3, digitalHandle);
                fclose(digitalHandle);
        }

        // Read value
        if((digitalHandle=fopen(GPIOValue, "r+"))!=NULL){
                fread(&readVal, 1, 1, digitalHandle);
                fclose(digitalHandle);
		return (int)(readVal - '0');
        }

	else{
		cout << "Cannot read pin" << pinNumber << endl;
		return 0;
	}
}
