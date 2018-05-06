#include <msp430.h> 
//for averaging ADC values
int adc3 = 0;
int adc4 = 0;
int buf3[10];
int buf4[10];
unsigned int index;

int Kp=3;
int Kd=0;
int Ki=0;
int error3 = 0;
int error4 = 0;
int error3Prev = 0;
int error4Prev = 0;
int proportional3 = 0;
int proportional4 = 0;
int derivative3 = 0;
int derivative4 = 0;
int accumulator3 = 0;
int accumulator4 = 0;
int integral3 = 0;
int integral4 = 0;
int pid3 = 0;
int pid4 = 0;
int tempLeft = 0;
int tempRight = 0;
int adc_out=0;
int scale_value = 8;

int control_type = 0;
int flag = 0;
int desired = 0; //ADC value
void PWMInit(void);
void sensing(void);
void starting(void);
void setMotor();
void ADCInit(void);
void updatePID(void);
void distToADC(int distance);
void setScaledMotor(int left, int right);
void pinInit(void);


/**
 * main.c
 */
int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Initialization
    pinInit();
    PWMInit();
    ADCInit();

    // Reset the speed of both motors
    setMotor(0,0);

    /*
    *   Compute desired value in ADC
    *   Max: 200 (closest)
    *   Min: 100 (furthest
    *
    *   desired = 200;
    */

    // Desired trailing distance
    int distance = 5; // in centimeters
    distToADC(distance); // Distance to ADC value

    // Sumo-bot constantly sense the environment
    while(1) //inifite loop
    {
        sensing();
    }
}


void starting(void)
{
    sensing();
}

/*
 * Enable the ADC converter to start sensing the environment
 * This function is called in main
 */
void sensing(void)
{
    ADC12CTL0 |= ADC12ENC;       // ADC12 enabled
    ADC12CTL0 |= ADC12SC;        // Start sampling/conversion
    __bis_SR_register(GIE);     // LPM0, ADC12_ISR will force exit

}

/*
 * Proportional - Integral - Derivative Controller of the robot
 * Is called in the Interrupt
 */
void updatePID(void){
    // Calculating the Error term
    error3Prev = error3; // Right motor
    error4Prev = error4; // Left motor
    error3 = desired - adc3;
    error4 = desired - adc4;

    // Calculating the Proportional term
    proportional3 = Kp * error3; // Right motor
    proportional4 = Kp * error4; // Left motor

    // Calculating the Derivative term
    derivative3 = Kd * (error3Prev-error3); // Right motor
    derivative4 = Kd * (error4Prev-error4); // Ledt motor

    // Calculating the accumulator for the Error term
    accumulator3 = accumulator3 += error3;
    accumulator4 = accumulator4 += error4;

    // Capping accumulator values
    if(accumulator3>50){ // Right motor
        accumulator3 = 50;
    }else if(accumulator3<-50){
        accumulator3 = -50;
    }

    if(accumulator4>50){  // Left motor
        accumulator4 = 50;
    }else if(accumulator4<-50){
        accumulator4 = -50;
    }

    // Calculating the Integral term
    integral3 = Ki * accumulator3; // Right motor
    integral4 = Ki * accumulator4; // Left motor

    // Adding the Proportional - Integral - Derivative temrs
    // up to create the PID controller
    pid3 = proportional3+integral3-derivative3;
    pid4 = proportional4+integral4-derivative4;

    //convert pid range to PWM range
    setScaledMotor(pid3,pid4);
}

/*
 * This function scale the PWM range, since the left motor is
 * slightly stronger than the right motor, making the robot
 * steer right.
 */
void setScaledMotor(int left, int right){
    //left limit -999 to 999
    //right limit -870 to 870
    //expected range -200 to 200
    tempLeft = (left*5/2)*scale_value;
    tempRight = ((right*9)/4)*scale_value;

    // 30 is minimum value (furthest)
    if (tempLeft < 30 && tempLeft > 0){
        tempLeft = 30;
    }
    if(tempRight < 30 && tempRight > 0){
        tempRight = 30;
    }

    if (tempLeft > -30 && tempLeft < 0){
        tempLeft = -30;
    }
    if(tempRight > -30 && tempRight < 0){
        tempRight = -30;
    }
    // 999 is maximum value (closest)
    if(tempLeft< -999){
        tempLeft = -999;
    }
    if(tempRight< -999){
        tempRight = -999;
    }

    if (tempLeft>999){
        tempLeft = 999;
    }
    if (tempRight>870){
        tempRight = 870;
    }
    //       Right      Left
    setMotor(tempLeft, tempRight);

}

/*
 * This function set the PWM value of the
 * two motors, hence control their speed
 */
void setMotor(int left, int right)
{
    if(right < 0) //Backwards, clock-wise
    {
        P2OUT |= BIT5; //Sets direction for H-bridge
        TA0CCR3 = (left * -1); //Speed of the motor CCR/1000
    }
    else if(right == 0) //stopping
    {
        TA0CCR3 = 0;
    }
    else //Forwards, counter clock-wise
    {
        P2OUT &= ~BIT5; //Sets direction for H-bridge
        TA0CCR3 = left; //Speed of the motor CCR/1000
    }

    if(left < 0)
    {
        P1OUT |= BIT5;
        TA2CCR1 = (right * -1);
    }
    else if(left == 0)
    {
        TA2CCR1 = 0;
    }
    else // forwards, clock-wise
    {
        P1OUT &= ~BIT5;
        TA2CCR1 = right;
    }
}

/*
 * Characterization code for the system
 * Convert the desired distance to ADC value
 * Using piece-wise functions
 */
void distToADC(int distance){
    // 0cm to 7cm range
    if (distance >= 0 && distance <= 7){
        adc_out = distance*-53.6 + 492.8;
    // 7cm to 12cm range
    } else if (distance > 7 && distance <= 12){
        adc_out = distance*-13.689 + 215.54;
    // 12cm to 30cm range
    } else if (distance > 12 && distance  <= 30){
        adc_out = distance*-1.8716 + 74.838;
    }
    // Default value
    else{
        adc_out = 290;
    }
}
void pinInit(void){
    P2DIR |= BIT0;  // Pin 2.0 initialization
    P2OUT |= BIT0;

    P2DIR |= BIT2;  // Pin 2.2 initialization
    P2OUT |= BIT2;

    P2DIR |= BIT5;
    P1DIR |= BIT5;

    P1SEL =0; //Select GPIO option
    P1DIR |=BIT0; //set Port 1.0 output ---LED
    P1OUT &= ~BIT0;  // LED OFF

    P4SEL =0; //Select GPIO option
    P4DIR |=BIT7; //set Port 4.7 output ---LED
    P4OUT &= ~BIT7;  // LED OFF

    P1DIR &=~(BIT1); //set Port 1.1 input --- pushbutton
    P1REN|=BIT1;//enable pull-up/pull-down resistor on
    P1OUT|=BIT1; //choose the pull-up resistor

    P1IE |=BIT1;//enable the interrupt on Port 1.1
    P1IES |=BIT1;//set as falling edge
    P1IFG &=~(BIT1);//clear interrupt flag
}
void PWMInit(void)
{
    P1DIR |= BIT4;      // Initialize PWM to output on P1.4
    P1SEL |= BIT4;

    P2DIR |= BIT4;      // Initialize PWM to output on P2.4
    P2SEL |= BIT4;

    TA0CCR0 =1000-1;
    TA2CCR0 =1000-1;
    TA0CCTL3 =OUTMOD_7;
    TA0CCR3 =999;
    TA2CCTL1 =OUTMOD_7;
    TA2CCR1 = 999;
    TA0CTL = TASSEL_2 + MC_1 + TACLR;
    TA2CTL = TASSEL_2 + MC_1 + TACLR;
}

void ADCInit(void)
{
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
    ADC12CTL0 |= ADC12ENC;
    ADC12CTL0 |= ADC12SC;
}

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
    case  8:break;                           // Vector  8:  ADC12IFG1
    case 10: break;                           // Vector 10:  ADC12IFG2
    case 12:break;                           // Vector 12:  ADC12IFG3
    case 14:
        // For every 10 ADC samples, we use 1 for stability of the system
        if(index < 10){     //adds new value to array for average of 10
            buf3[index] = ADC12MEM3; // buffer for right motor
            buf4[index] = ADC12MEM4;
            index++;
        }
        else{     //computes average of 10 and transmits value; resets array index
            long average3 = 0;
            long average4 = 0;
            int i = 0;
            for(i = 0; i < 10 ; i ++){
                average3 += buf3[i];
                average4 += buf4[i];
            }
            average3 /= 10;
            average4 /= 10;
            index=0;
            flag=1;
            // -120 for Full vs Weak
            adc3 = average3 ; //changes duty cycle
            adc4 = average4 ; //changes duty cycle

            // Calling the PID controller function
            updatePID();
        }

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

/*
 * Button Interrupt that control the Controller Type of the robot
 * There are 4 modes in total: only Proportional, Proportional-Integral,
 * Proportional-Derivative, and all Proportional-Integral-Derivative
 */
#pragma vector=PORT1_VECTOR
__interrupt void PORT_1(void)
{
    P1IE &= ~BIT1; // Disable interrupt

    //Debounce 1
    __delay_cycles(1);

    //Debounce 2
    TA1CTL = TASSEL_1 + MC_1 + ID_1; //Set up Timer A, Count up, divider 2
    TA1CCTL0 = 0x10; // Set up compare mode for CCTL
    TA1CCR0 = 2000; // Duration at which the interrupt is disable
                    // Duration 2000/16kHz = 1/8 sec.
    P1IFG &=~(BIT1); // Clear flag

    // Loop back to control type 0
    if (control_type > 3){
        control_type = 0;
    }

    switch(control_type){
    case 0: //Only PI
        P4OUT |= BIT7; // Turn 4.7 on for 01
        P1OUT &= ~BIT0; // Turn 1.0 on
        Ki =1;
        scale_value = 5;
        break;
    case 1: // PD
        P1OUT |= BIT0; // Turn 1.0 on for 10
        P4OUT &= ~BIT7; // Turn 4.7 off
        Ki = 0;
        Kd = 1;
        scale_value = 8;
        break;
    case 2: //PID
        P4OUT |= BIT7; // Turn 4.7 on for 11
        P1OUT |= BIT0; // Turn 1.0 on for
        Ki = 1;
        Kd = 1;
        scale_value = 1;
        break;
    case 3: //Only P
        P4OUT &= ~BIT7; // Turn 4.7 on for 11
        P1OUT &= ~BIT0; // Turn 1.0 on for
        Ki = 0;
        Kd = 0;
        scale_value = 8;
        break;
    default:
        break;
    }
    control_type += 1;


}
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A0(void)
{
    P1IE |= BIT1; //Enable interrupt again.
}

