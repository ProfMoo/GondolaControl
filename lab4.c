// lab4.c
// Alex Elias, Shane O'Brien, Kevin Yun

#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

// INITIALIZATIONS
void portInit(void); // TODO
void interruptInit(void);
void PCAInit(void);
void SMB0Init(void);
void ADCInit(void);
void speedInit(void);
void steeringCalibration(void);

// ISR
void PCA_ISR ( void ) __interrupt 9;


// HIGH-LEVEL FUNCTIONS
void updateSetScreen(void);
void readKeypad(void);
void readKeypadManualMode(void);
unsigned short readRanger(void);
unsigned short readCompass(void);
unsigned char readPot(void);
unsigned int readBattery(void);
void updateRunScreen(void);
void setSteering(void);
void setSpeed(void);
void printCSV(void);

// HELPER FUNCTIONS
void pause(void);


#define PCA_START 28672 // 20ms period

#define TURN_DISTANCE 55
#define STOP_DISTANCE 20

unsigned short SERVO_PW_MIN = 2500; // .0009 x (22.1184e6 / 12) = 1658.8
unsigned short SERVO_PW_CENTER = 2765; // .0015 x (22.1184e6 / 12) = 2764.8
unsigned short SERVO_PW_MAX = 3200; // .0021 x (22.1184e6 / 12) = 3870.7
unsigned short SERVO_PW = 2765; // Center

#define SPEED_PW_MIN 2028 // 1.1ms * (22.1184 MHz / 12) = 24 330.24
#define SPEED_PW_CENTER 2765 // 1.5ms * (22.1184 MHz / 12) = 33 177.6
#define SPEED_PW_MAX 3502 // 1.9ms * (22.1184 MHz / 12) = 42 024.96
unsigned short SPEED_PW = 2765; // Center

char canUpdateScreen = 0, canReadKeypad = 0, canReadRanger = 0;
char canReadCompass = 0, canReadBattery = 0, inManualMode = 0;

char updateScreenCount = 0, readKeypadCount = 0, readRangerCount = 0;
char readCompassCount = 0, readBatteryCount = 0, PCAOverflows = 0;
char numButtonsPressed = 0, hasTurned = 0;

unsigned short distance = 60, rangerError = 0, heading = 0,
tempDesiredHeading = 0;

unsigned char  potValue = 128, rangerGain = 1,
compassGain = 1, displayBlink = 0, waitForButtonRelease = 0;

unsigned int batteryVoltage = 0;

short compassError = 0, desiredHeading = 90;

__sbit __at 0xB7 slideSwitch; // Pin 3.7, 0 when closed


main(void) {
    Sys_Init();
    putchar(' ');
    portInit();
    interruptInit();
    PCAInit();
    SMB0Init();
    ADCInit();
    
    speedInit();
    steeringCalibration();
   
    while(1) {
        if (slideSwitch) { // set paramaters mode
            if (hasTurned) {
                hasTurned = 0;
                desiredHeading -= 90;
                if (desiredHeading < 0)
	                desiredHeading += 360;
	            distance=100; // so it doesn't think it's close when it starts
            }
            if (canReadBattery) {
                canReadBattery = 0;
                batteryVoltage = readBattery();
            }
            //printf("pot value: %u/r/n", readPot());
            SERVO_PW = SERVO_PW_CENTER;
            SPEED_PW = SPEED_PW_CENTER;
            if (canUpdateScreen) {
                canUpdateScreen = 0;
                updateSetScreen();
            }
            if (canReadKeypad) {
                canReadKeypad = 0;
                if (!inManualMode)
                    readKeypad();
                else
                    readKeypadManualMode();
            }
        }
        else { // run mode
            if (canReadBattery) {
                canReadBattery = 0;
                batteryVoltage = readBattery();
            }
            if (canReadRanger) {
                canReadRanger = 0;
                distance = readRanger();
            }
            if (canReadCompass) {
                canReadCompass = 0;
                heading = readCompass();
				printCSV();
            }
            setSteering();
            setSpeed();
            if (canUpdateScreen) {
                canUpdateScreen = 0;
                updateRunScreen();
            }
        }
    }
}


// INITIALIZATIONS /////////////////////////////////////////////////////////////

void portInit(void) {
    XBR0 = 0x27; // Enable UART0, SMB0
    // TODO Set input pin modes
    // Port 1
    P3MDOUT |= 0x05; // Outputs: P1.0, P1.2 (PWM outputs)
    P3MDOUT &= 0x7F; // Digital Inputs: P3.7 (Slide Switch)
    P1MDIN &= 0x3F; // Analog Inputs: P1.6, P1.7 (Pot, 12v divider)
    P3 |= 0x80; // set digital inputs to high impedence
    P1 |= 0xC0; // set analog inputs to high impedence
}

void interruptInit(void) {
    EIE1 |= 0x08; // enable PCA interrupt
    EA = 1; // enable global interrupt
}

void PCAInit(void){
    PCA0CPM0 = 0xC2;    // CEX0 to 16-bit
    PCA0CPM2 = 0xC2;    // CEX2 to 16-bit
    PCA0MD = 0x81;      // SYSCLK/12, enable CF interrupts, suspend when idle
    PCA0CN |= 0x40;     // enable PCA
}

void SMB0Init(void){
    SMB0CR = 0x93;      // Set SCL to 100KHz
    ENSMB = 1;          // Enable SMBUS0
}

void ADCInit(void) {
    REF0CN = 0x03; // set internal reference 
    ADC1CF = 0x01; // set to gain of 1
    ADC1CN = 0x80; // enable converter
    AMX1SL = 7; // connect multoplexer to pot on pin 1.7
}

// Set the pulsewidth to 1.5ms and leave it for about 1 second
void speedInit(void) {
    printf("Starting motor controller initialization\r\n");
    PCAOverflows = 0;
    while (PCAOverflows < 60); // 1s / 20ms
    printf("Completed motor controller initialization\r\n");
}

void steeringCalibration(void) {
    char input;
    printf("Steering Calibration\r\n");
    
    printf("Center Calibration: press l and r to move, press d when done\r\n");
    SERVO_PW = SERVO_PW_CENTER;
    input = getchar();
    while (input != 'd') {
        if (input == 'l') {
            SERVO_PW -= 10;
        }
        else if (input == 'r') {
            SERVO_PW += 10;
        }
        input = getchar();
    }
    SERVO_PW_CENTER = SERVO_PW;
    printf("SERVO_PW_CENTER: %u\r\n", SERVO_PW_CENTER);
    
    printf("Left Calibration: press l and r to move, press d when done\r\n");
    SERVO_PW = SERVO_PW_MIN;
    input = getchar();
    while (input != 'd') {
        if (input == 'l') {
            SERVO_PW -= 10;
        }
        else if (input == 'r') {
            SERVO_PW += 10;
        }
        input = getchar();
    }
    SERVO_PW_MIN = SERVO_PW;
    printf("\r\nSERVO_PW_MIN: %u\r\n", SERVO_PW_MIN);
    
    printf("Right Calibration: press l and r to move, press d when done\r\n");
    SERVO_PW = SERVO_PW_MAX;
    input = getchar();
    while (input != 'd') {
        if (input == 'l') {
            SERVO_PW -= 10;
        }
        else if (input == 'r') {
            SERVO_PW += 10;
        }
        input = getchar();
    }
    SERVO_PW_MAX = SERVO_PW;
    printf("\r\nSERVO_PW_MAX: %u\r\n", SERVO_PW_MAX);
}

// ISR /////////////////////////////////////////////////////////////////////////

void PCA_ISR ( void ) __interrupt 9 {
    if (CF) {
        updateScreenCount++;
        readKeypadCount++;
        readRangerCount++;
        readCompassCount++;
        readBatteryCount++;
        PCAOverflows++;
        
        PCA0 = PCA_START;
        PCA0CP0 = 0xFFFF - SERVO_PW;
        PCA0CP2 = 0xFFFF - SPEED_PW;
        
        if (updateScreenCount > 16) { // 3 Hz: (1/3)s / 20ms = 16.67
            canUpdateScreen = 1;
            updateScreenCount = 0;
        }
        if (readKeypadCount > 2) {
            canReadKeypad = 1;
            readKeypadCount = 0;
        }
        if (readRangerCount > 5) {
            canReadRanger = 1;
            readRangerCount = 0;
        }
        if (readCompassCount > 3) {
            canReadCompass = 1;
            readCompassCount = 0;
        }
        if (readBatteryCount > 50) { // once a second
            canReadBattery = 1;
            readBatteryCount = 0;
        }
        
        CF = 0; //clear interrupt flag
    }
    else PCA0CN &= 0xC0;    //all other type 9 interrupts
}

// HIGH-LEVEL FUNCTIONS ////////////////////////////////////////////////////////

void updateSetScreen(void) {
    //  ____________________
    // |Desired Heading: XXX| (Blinking tempDeriredHeading if inManualMode)
    // |Compass Gain:    XXX|
    // |Ranger Gain:     XXX|
    // |                    |
    //  ____________________
    lcd_clear();
    if (inManualMode) {
        if (displayBlink) {
            lcd_print("Desired Heading:\n");
            printf("Desired Heading:\r\n");
            displayBlink = 0;
        }
        else {
            lcd_print("Desired Heading:%u\n", tempDesiredHeading);
            printf("Desired Heading: %u\r\n", tempDesiredHeading);
            displayBlink = 1;
        }
    }
    else {
        lcd_print("Desired Heading:%u\n", desiredHeading);
        printf("Desired Heading: %u\r\n", desiredHeading);
    }
    lcd_print("Compass Gain:   %u\n", compassGain);
    lcd_print("Ranger Gain:    %u\n", rangerGain);
    lcd_print("Voltage:%umv", batteryVoltage);
    
	pause();
	printf("Compass Gain:    %u\r\n", compassGain);
	pause();
    printf("Ranger Gain:     %u\r\n", rangerGain);
    pause();
    printf("Voltage:%umv\r\n\n", batteryVoltage);
}

void readKeypad(void) {
    char buttonPressed = 0;
    char KPButtonPressed = read_keypad();
    
    if (KPButtonPressed == -1) {
        waitForButtonRelease = 0;
    }
    else if (!waitForButtonRelease) {
        waitForButtonRelease = 1;
        buttonPressed = KPButtonPressed;
    }
    if (buttonPressed == 0) { // Now check serial port if keypad key not pressed
        if (RI0) { // UART0 return interrupt flag
            RI0 = 0;
            buttonPressed = SBUF0; // Input character from UART0
        }
    }
    if (buttonPressed){
        // 1: compassGain++    2: 0deg         3: rangerGain++
        // 4: 270deg           5: manual mode  6: 90deg
        // 7: compassGain--    8: 180deg       9: rangerGain--
        // *: nothing          0: nothing      #: nothing
        if (buttonPressed == '1') compassGain++;
        else if (buttonPressed == '2') desiredHeading = 0;
        else if (buttonPressed == '3') rangerGain++;
        else if (buttonPressed == '4') desiredHeading = 270;
        else if (buttonPressed == '5') {
            tempDesiredHeading = 0;
            inManualMode = 1;
        }
        else if (buttonPressed == '6') desiredHeading = 90;
        else if (buttonPressed == '7') compassGain--;
        else if (buttonPressed == '8') desiredHeading = 180;
        else if (buttonPressed == '9') rangerGain--;
    }
} 

void readKeypadManualMode(void){
    // Read from both keypad and serial port
    char buttonPressed = 0;
    char KPButtonPressed = read_keypad();
    
    if (KPButtonPressed == -1) {
        waitForButtonRelease = 0;
    }
    else if (!waitForButtonRelease) {
        waitForButtonRelease = 1;
        buttonPressed = KPButtonPressed;
    }
    if (buttonPressed == 0) { // Now check serial port if keypad key not pressed
        if (RI0) { // UART0 return interrupt flag
            RI0 = 0;
            buttonPressed = SBUF0; // Input character from UART0
        }
    }
    if (buttonPressed >= '0' && buttonPressed <= '9'){
        numButtonsPressed++;
        tempDesiredHeading *= 10;
        tempDesiredHeading += buttonPressed - '0';
    }
    if (numButtonsPressed == 3) {
        desiredHeading = tempDesiredHeading;
        inManualMode = numButtonsPressed = 0;
    }
}

unsigned short readRanger(void) {
    unsigned char Data[2];
    unsigned char command[1] = {0x51}; // Ranging Mode - Result in centimeters
	unsigned short range = 0;
	unsigned char addr = 0xE0; // the address of the ranger is 0xE0
	
    i2c_read_data(addr, 2, Data, 2); // read two bytes, starting at reg 2
    range = (((unsigned short)Data[0] << 8) | Data[1]);
    i2c_write_data(addr, 0, command, 1);
    
    return range; 
}

unsigned short readCompass(void) {
    unsigned char addr = 0xC0; // the address of the sensor, 0xC0 for the compass
    unsigned char Data[2]; // Data is an array with a length of 2
    unsigned short heading; // the heading returned in degrees between 0 and 3599
    i2c_read_data(addr, 2, Data, 2); // read two byte, starting at reg 2
    heading = (((unsigned short)Data[0] << 8) | Data[1]); //combine the two values
    //heading has units of 1/10 of a degree
    return heading; // the heading returned in degrees between 0 and 3599
}

unsigned char readPot(void) {
    ADC1CN &= ~0x20; // Clear conversion complete flag from previous time
    ADC1CN |= 0x10; // Start conversion
    while((ADC1CN & 0x20) == 0x00); // Wait for conversion to complete
    return ADC1;
}

unsigned int readBattery(void) {
    AMX1SL = 6; // Connect multiplexer to pin 1.6, connected to 12v divider
    pause(); // Wait for voltage to stabilize
    ADC1CN &= ~0x20; // Clear conversion complete flag from previous time
    ADC1CN |= 0x10; // Start conversion
    while((ADC1CN & 0x20) == 0x00); // Wait for conversion to complete
    AMX1SL = 7; // Switch back to pot
    return ADC1 * 62; //  /255 * 2.4v * (R1+R2)/R2 * 1000mv/V
}

void updateRunScreen(void){
    signed long servoPercent, speedPercent;
    servoPercent = (long)(SERVO_PW) - SERVO_PW_CENTER;
    servoPercent *= 200;
    servoPercent /= (SERVO_PW_MAX - SERVO_PW_MIN);
    
    speedPercent = (long)(SPEED_PW) - SPEED_PW_CENTER;
    speedPercent *= 200;
    speedPercent /= (SPEED_PW_MAX - SPEED_PW_MIN);

    //  ____________________
    // |Hdng: XXX Dst: XX   |
    // |Srvo: XXX% Spd: XXX%|
    // |CmpsG: XXX Turn: XXX|
    // |desiredH: XXXX      |
    //  ____________________
    
    lcd_clear();
    lcd_print("Hdng:%u Dst:%u\n", heading, distance);
    lcd_print("Srvo:%ld%% Spd:%ld%%\n",  servoPercent, speedPercent);
	//lcd_print("Srvo:%d Spd:%d\n",  SERVO_PW, SPEED_PW);
    lcd_print("CmpsG:%u Turn:%u\n", compassGain, hasTurned);
    lcd_print("desiredH: %u", desiredHeading);
    //printf("Srvo:%ld%% Spd:%ld%%\r\n",  servoPercent, speedPercent);
}

void setSteering(void) {
	unsigned int tempServoPW;
	
	if (!hasTurned && distance < TURN_DISTANCE) {
	    desiredHeading += 90;
	    if (desiredHeading >= 360)
	        desiredHeading -= 360;
	    hasTurned = 1;
	}
	
    compassError = desiredHeading*10 - heading;
    
    if (compassError > 1800)
        compassError -= 3600;
    else if (compassError < -1800)
        compassError += 3600;

	compassError /= 10;
    
    tempServoPW = SERVO_PW_CENTER + compassGain * compassError;
    
    if (tempServoPW < SERVO_PW_MIN)
        SERVO_PW = SERVO_PW_MIN;
    else if (tempServoPW > SERVO_PW_MAX)
        SERVO_PW = SERVO_PW_MAX;
    else
        SERVO_PW = tempServoPW;
}

void setSpeed(void) {
    if (hasTurned && distance < STOP_DISTANCE) {
	    SPEED_PW = SPEED_PW_CENTER;
	}
    else  // 0~255 -> 2028~3502
        SPEED_PW = 2028 + ((unsigned int)readPot() * 58) / 10; // (3502 - 2028)) / 255
		//SPEED_PW = readPot();
}

void printCSV(void){
    printf("%d,%u,%u,%u,%u,%u,%u,%u\r\n", compassError, SERVO_PW, heading,
    distance, batteryVoltage, rangerGain, compassGain, SPEED_PW);
}

// HELPER FUNCTIONS ////////////////////////////////////////////////////////////

void pause(void) {
    PCAOverflows = 0;
    while (PCAOverflows < 1);
}