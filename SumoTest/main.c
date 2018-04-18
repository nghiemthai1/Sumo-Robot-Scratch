//MattttttttLawrence
#include <msp430.h>

//Initialize Sensor Thresholds.
const int PHOTORESISTOR_THRESH1 = 200;
const int PHOTORESISTOR_THRESH2 = 200;
const int IRSENSOR_THRESH1 = 40;
const int IRSENSOR_THRESH2 = 40;
int sensorThreshHolds[4] = {PHOTORESISTOR_THRESH1, PHOTORESISTOR_THRESH2, IRSENSOR_THRESH1, IRSENSOR_THRESH2};

//Initialize Speed constants and turn duration.
int MAX = 999;
int TURN_SPEED = 999;
unsigned int TURN_TIME = 500000;
int attackSpeedLeft, attackSpeedRight;

//Initialize variable that the program can read to see sensor values.
int adc_value[4] = {0,0,0,0};

//define button and variables for UART remote control mode.
#define BUTTON BIT0;
int fightMode = -1;
int direction = 0;

void attack();
void backAway();
int sensorCheck();
int setSensor();
int readSensor();
void standBy();

void ADCInit(void)
{

    /*Configure ADC*/
    /*ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
    ADC12CTL1 =  ADC12CSTARTADD_1 + ADC12SHP + ADC12CONSEQ_1;                     // Use sampling timer
    ADC12CTL0 |= BIT4 + BIT5 + BIT6;
    ADC12MCTL1 = ADC12INCH_1;
    ADC12MCTL2 = ADC12INCH_2;
    ADC12MCTL3 = ADC12INCH_3;
    ADC12MCTL4 = ADC12INCH_4 + ADC12EOS;
    ADC12IE |= BIT1 + BIT2 + BIT3 + BIT4;                           // Enable interrupt
    ADC12CTL0 |= ADC12ENC;*/
    ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT02; // Turn on ADC12, set sampling time
    ADC12CTL1 = ADC12SHP+ADC12CONSEQ_1;       // Use sampling timer, single sequence
    ADC12MCTL0 = ADC12INCH_0;                 // ref+=AVcc, channel = A0
    ADC12MCTL1 = ADC12INCH_1;                 // ref+=AVcc, channel = A1
    ADC12MCTL2 = ADC12INCH_2;                 // ref+=AVcc, channel = A2
    ADC12MCTL3 = ADC12INCH_3;
    ADC12MCTL4 = ADC12INCH_4 + ADC12EOS;        // ref+=AVcc, channel = A3, end seq.
    ADC12IE = 0x10;                           // Enable ADC12IFG.3
    ADC12CTL0 |= ADC12ENC;                    // Enable conversions
    /*Sets ADC Pin to NOT GPIO*/
    P6SEL |= BIT1 + BIT2 + BIT3 + BIT4;                            // P6.0 ADC option select
    P6DIR &= ~(BIT1 + BIT2 + BIT3 + BIT4);
    P6REN |= BIT1 + BIT2 + BIT3 + BIT4;
    P6OUT &= ~(BIT1 + BIT2 + BIT3 + BIT4);
}

void UARTInit(void)
{
    /*Configures UART*/
    P3SEL |= BIT3;          // UART TX
    P3SEL |= BIT4;          // UART RX
    UCA0CTL1 |= UCSWRST;    // Resets state machine
    UCA0CTL1 |= UCSSEL_2;   // SMCLK
    UCA0BR0 = 6;            // 9600 Baud Rate
    UCA0BR1 = 0;            // 9600 Baud Rate
    UCA0MCTL |= UCBRS_0;    // Modulation
    UCA0MCTL |= UCBRF_13;   // Modulation
    UCA0MCTL |= UCOS16;     // Modulation
    UCA0CTL1 &= ~UCSWRST;   // Initializes the state machine
    UCA0IE |= UCRXIE;
}

void pinInit(void)
{
    P1DIR |= BIT2 + BIT3 + BIT4 + BIT5; // for motors
    P1DIR &= ~BIT0;                            // Set P1.0 to output direction
    P1OUT &= ~BIT0;

    P1REN |= BIT1;                            // Enable P1.1 internal resistance
    P1OUT |= BIT1;                            // Set P1.1 as pull-Up resistance
    P1IES &= ~BIT1;                           // P1.1 Lo/Hi edge
    P1IFG &= ~BIT1;                           // P1.1 IFG cleared
    P1IE |= BIT1;                             // P1.1 interrupt enabled
}

void timerInit(void)
{
    P2DIR |= BIT4+BIT5;                       // P1.2 and P1.3 output for motors
    P2SEL |= BIT4+BIT5;                       // P1.2 and P1.3 options select for motors
    TA2CCR0 = 1000 - 1;                          // PWM Period
    TA2CCTL1 = OUTMOD_7;                      // CCR1 reset/set
    TA2CCR1 = 384;                            // CCR1 PWM duty cycle
    TA2CCTL2 = OUTMOD_7;                      // CCR2 reset/set
    TA2CCR2 = 128;                            // CCR2 PWM duty cycle
    TA2CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up mode, clear TAR
}

void motorPWM(int left, int right)
//'left' and 'right' are the input speed
{
    if(left < 0) //backwards, clock-wise
    {
        P1OUT &= ~BIT4;
        P1OUT |= BIT5; // Sets direction for H-bridge
        TA2CCR2 = (left * -1);//Speed of motor CCR/1000
    }
    else if(left == 0) //stopping
    {
        P1OUT &= ~BIT4;
        P1OUT &= ~BIT5;// Sets direction for H-bridge
        TA2CCR2 = 0;//Speed of motor CCR/1000
    }
    else // forwards, counter clock-wise
    {
        P1OUT |= BIT4;
        P1OUT &= ~BIT5;// Sets direction for H-bridge
        TA2CCR2 = left;//Speed of motor CCR/1000
    }

    if(right < 0) //backwards, counter clock-wise
    {

        P1OUT &= ~BIT2;
        P1OUT |= BIT3; // Sets direction for H-bridge
        TA2CCR1 = (right * -1);//Speed of motor CCR/1000
    }
    else if(right == 0) //stopping
    {
        P1OUT &= ~BIT2;
        P1OUT &= ~BIT3;// Sets direction for H-bridge
        TA2CCR1 = 0;//Speed of motor CCR/1000
    }
    else // forwards, clock-wise
    {
        P1OUT |= BIT2;
        P1OUT &= ~BIT3;// Sets direction for H-bridge

        TA2CCR1 = right;
    }
}


// Get the ADC value using ADC interrupt
int readSensor(int SensorToRead)
{
    __delay_cycles(10000);
    ADC12CTL0 |= ADC12ENC;
    ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
    __bis_SR_register(LPM4_bits + GIE);     // LPM0, ADC12_ISR will force exit


    return adc_value[SensorToRead];
}

// Check the status of all 4 sensors
int sensorCheck()
{
    if(readSensor(0) > sensorThreshHolds[0])
    {
        return 0;
    }
    else if(readSensor(1) > sensorThreshHolds[1])
    {
        return 1;
    }
    else if(readSensor(2) > sensorThreshHolds[2])
    {
        return 2;
    }
    else if(readSensor(3) > sensorThreshHolds[3])
    {
        return 3;
    }



    return -1;
}

// Attach strategy for the robot
void attack()
{
    int attackSpeedLeft = 0, attackSpeedRight = 0; //Initialize attack speed
    int sensorValue = sensorCheck(); // Check each sensor's value
    if(sensorValue == -1) // If there is no significant changes, do nothing
        return;
    else if(sensorValue == 0 || sensorValue == 1) // The sensors on the bottom
    {
        backAway(sensorValue); //Back the bot away
        return;
    }
    else if(sensorValue == 2 || sensorValue == 3) // The sensors on the top
    {
        attackSpeedLeft = MAX;
        attackSpeedRight = MAX;
    }
    // Call motorPWM mode to tell how the robot should move
    motorPWM(attackSpeedLeft,attackSpeedRight);
}

// Back the robot away if it on the edge of the Dojo
void backAway(int leftOrRight)
{
    motorPWM(-999, -999);
    __delay_cycles(300000);
    // Choose which way to turn
    switch(leftOrRight)
    {
    case 0: // Left
        motorPWM(999, -999);
        break;
    case 1: // Right
        motorPWM(-999, 999);
        break;
    }



    // How long the robot's going turning for
    __delay_cycles(1000000);


    motorPWM(999, 999);
    __delay_cycles(500000);

    // How long the robot's going to go forward before going
    // into stand-by mode.

}

// Default mode of the robot
void standBy()
{
    int sensorValue = sensorCheck(); // Check each sensor's value
    if(sensorValue == -1) // If there is no significant changes, do nothing
    {
        motorPWM(-999, 999); // Spin
        return;
    }
    else if(sensorValue == 0 || sensorValue == 1) // The sensors on the bottom
    {
        backAway(sensorValue); //Back the bot away
        return;
    }
    else if(sensorValue == 2 || sensorValue == 3) // The sensors on the top
    {
        attack();
        return;
    }
}
void remoteControl()
{
    if(direction == 1)
    {
        motorPWM(999,999);
        __delay_cycles(200000);
        direction = 0;
    }
    else if(direction == 2)
    {
        motorPWM(-999,-999);
        __delay_cycles(200000);
        direction = 0;
    }
    else if(direction == 3)
    {
        motorPWM(-999,999);
        __delay_cycles(200000);
        direction = 0;
    }
    else if(direction == 4)
    {
        motorPWM(999,-999);
        __delay_cycles(200000);
        direction = 0;
    }
    else{
        motorPWM(0,0);
    }

}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    //Initializations
    pinInit();
    timerInit();
    UARTInit();
    ADCInit();
    __bis_SR_register(GIE);
    __enable_interrupt();


    fightMode = -1;
    while(1)
    {
        if(fightMode == 1)
        {
            __delay_cycles(5000000);
            standBy();
        }
        if(fightMode == 0)
        {
            remoteControl();
        }
    }
}
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    switch( __even_in_range( P1IV, P1IV_P1IFG7 )) {
    case P1IV_P1IFG1:                                       // Pin 1 (button 2)
        if(fightMode != 1)
            {
                fightMode = 1;
            }
            else
            {
                fightMode =0;
            }
        break;
    default:   _never_executed();

    }

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{

    while (!(UCA0IFG&UCTXIFG)); // USCI_A0 TX buffer ready?
    if(UCA0RXBUF == 119)//w
    {
        direction = 1;
        UCA0TXBUF = UCA0RXBUF;
    }
    else if(UCA0RXBUF == 115)//s
    {
        direction = 2;
        UCA0TXBUF = UCA0RXBUF;
    }
    else if(UCA0RXBUF == 100)//a
    {
        direction = 3;
        UCA0TXBUF = UCA0RXBUF;
    }
    else if(UCA0RXBUF == 97)//d
    {
        direction = 4;
        UCA0TXBUF = UCA0RXBUF;
    }

    else
        direction = 0;
    UCA0TXBUF = UCA0RXBUF; // Show that the wrong button is pressed
}

/*ADC Interrupt*/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{

    switch(__even_in_range(ADC12IV,34))
    {
    case  0: break;                           // Vector  0:  No interrupt
    case  2: break;                           // Vector  2:  ADC overflow
    case  4: break;                           // Vector  4:  ADC timing overflow
    case  6: break;                                  // Vector  6:  ADC12IFG0
    case  8:
        adc_value[0] = ADC12MEM1; //changes duty cycle
        adc_value[1] = ADC12MEM2; //changes duty cycle
        adc_value[2] = ADC12MEM3; //changes duty cycle
        adc_value[3] = ADC12MEM4; //changes duty cycle
        __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
        break;                           // Vector  8:  ADC12IFG1
    case 10: break;                           // Vector 10:  ADC12IFG2
    case 12:
        adc_value[0] = ADC12MEM1; //changes duty cycle
        adc_value[1] = ADC12MEM2; //changes duty cycle
        adc_value[2] = ADC12MEM3; //changes duty cycle
        adc_value[3] = ADC12MEM4; //changes duty cycle
        __bic_SR_register_on_exit(LPM0_bits);
        break;                           // Vector 12:  ADC12IFG3
    case 14:
        adc_value[0] = ADC12MEM1; //changes duty cycle
        adc_value[1] = ADC12MEM2; //changes duty cycle
        adc_value[2] = ADC12MEM3; //changes duty cycle
        adc_value[3] = ADC12MEM4; //changes duty cycle
        __bic_SR_register_on_exit(LPM0_bits);
        break;                           // Vector 14:  ADC12IFG4
    case 16: break;                           // Vector 16:  ADC12IFG5
    case 18: break;                           // Vector 18:  ADC12IFG6
    case 20: break;                           // Vector 20:  ADC12IFG7
    case 22: break;                           // Vector 22:  ADC12IFG8
    case 24: break;                           // Vector 24:  ADC12IFG9
    case 26: break;                           // Vector 26:  ADC12IFG10
    case 28: break;                           // Vector 28:  ADC12IFG11
    case 30: break;                           // Vector 30:  ADC12IFG12
    case 32: break;                           // Vector 32:  ADC12IFG13
    case 34: break;                           // Vector 34:  ADC12IFG14
    default: break;
    }
}
