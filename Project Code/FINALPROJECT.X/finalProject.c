#include <xc.h>
#include <pic18f4520.h>
#include <stdio.h>
// CONFIG
#pragma config OSC  = INTIO67,WDT=OFF,LVP=OFF
//#pragma config OSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDT = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRT = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

#pragma PBADEN = 1 //set AN0~AN12 as analog input
#define _XTAL_FREQ 4000000
//***Define the signal pins of all four displays***//
#define s1 RA7
#define s2 RA6
#define s3 RC0
#define s4 RC1
//***End of definition**////

//#define Trigger_Pulse LATD0     /* PORTD.0 pin is connected to Trig pin of HC-SR04 */

int tmr3Mode = 0;   //0: setting, 1: running
int count2sec = 0;

float Distance;

void Trigger_Pulse_10us();
void adc_init(void);
void ccp2_init(void);
void tmr3_init(int mode);
int array[8];
int index;

int step_frequency;
int sec_10;

int one;
int two;
int three;

int f_one;
int f_two;
int f_three;

int count;
int eye;

int upper;
int lower;

int step;

//for breath LED
PWM1_Init(long setDuty);
PWM1_Duty(unsigned int duty);
PWM1_Start();

//for buzzer
void tmr0_init();
void tmr0_onoff(int o);

//RGB LED
int R=0, G=0, B=0;      //set color

//breath LED
float t=0;              //caculate breath LED delay time
int breathTime = 0;     //convert float t to int
int flagBreath = 0;     //control duty cycle increase or decrease
int setBreath = 0;      //duty cycle value, from 0 to 500, 500 to 0
int addBreath = 0;

//buzzer
int flagBuzzer = 0;
int buzzerF = 0;
int first10 = 0;


void showPace(int i) {
    int a,b,c,d,e,f,g,h; //just variables
    int seg[]= 
    {
        0b11000000, //Hex value to display the number 0
        0b11111001, //Hex value to display the number 1
        0b10100100, //Hex value to display the number 2
        0b10110000, //Hex value to display the number 3
        0b10011001, //Hex value to display the number 4
        0b10010010, //Hex value to display the number 5
        0b10000010, //Hex value to display the number 6
        0b11011000, //Hex value to display the number 7
        0b10000000, //Hex value to display the number 8
        0b10010000  //Hex value to display the number 9
    }; //End of Array for displaying numbers from 0 to 
     //***Splitting "i" into four digits***//  
        a=i%10;//4th digit is saved here
        b=i/10;
        c=b%10;//3rd digit is saved here
        d=b/10;
        e=d%10; //2nd digit is saved here
        f=d/10;
        g=f%10; //1st digit is saved here
        h=f/10;
        //***End of splitting***//
        //LATD = seg[0];//Turn ON display 1 and print 4th digit
        LATD=seg[g];s1=1; //Turn ON display 1 and print 4th digit
        __delay_ms(2);s1=0;     //Turn OFF display 1 after 5ms delay
        LATD=seg[e];s2=1; //Turn ON display 2 and print 3rd digit
        __delay_ms(2);s2=0;     //Turn OFF display 2 after 5ms delay
        LATD=seg[c];s3=1; //Turn ON display 3 and print 2nd digit
        __delay_ms(2);s3=0;     //Turn OFF display 3 after 5ms delay
        LATD=seg[a];s4=1; //Turn ON display 4 and print 1st digit
          __delay_ms(2);s4=0;     //Turn OFF display 4 after 5ms delay
//        if(flag>=1){
//            i++;flag=0; //only if flag is hundred "i" will be incremented 
//        }
//        flag++; //increment flag for each flash
}


void initialize_7() {

    TRISAbits.RA7 = 0;
    TRISAbits.RA6 = 0;
    
    TRISCbits.RC0 = 0;
    TRISCbits.RC1 = 0;
    PORTCbits.RC0 = 0;
    PORTCbits.RC1 = 0;
    TRISD=0x00;
    LATD =0x00;
}


void __interrupt(high_priority) Hi_ISR(void)
{
    //deal ccp2 interrupt and adc interrupt
    if(PIR1bits.ADIF){//AD conversion finish
        PIR1bits.ADIF = 0;  //reset A/D converter flag bit
        ADCON0bits.ADON = 1;//  enable converter
        array[index] = ADRES;  //ad result
        if(index <  8){
            index = index+1;
        }
        else{
            index = 0;
        }
        
        if(f_one == 1){
            one = ADRES;
            ADCON0bits.CHS = 0b0001;
            f_one = 0;
            f_two = 1;
        }
        else if(f_two == 1){
            two = ADRES;
            ADCON0bits.CHS = 0b0010;
            f_two = 0;
            f_three = 1;
        }
        else{
            three = ADRES;
            ADCON0bits.CHS = 0b0000;
            f_three = 0;
            f_one = 1;
        }
    }

    else if(PIR2bits.CCP2IF){ //special event triger
        PIR2bits.CCP2IF = 0;
        CCPR2 = 31250;        // 1M/4 = 250000 , 0.125*250000 = 31250,    65535-31250 = 34285
    }
    
    if(PIR2bits.TMR3IF == 1){
        PIR2bits.TMR3IF = 0;
        TMR3 = 3035;
        count2sec++;
        if(count2sec == 4){     //2 sec
            count2sec = 0;
            first10=first10+2;
            if(first10>=10)
            {
                first10 = 10;
            }
            sec_10++;
        }
    }
    if(sec_10 == 2){
        sec_10 = 0;
        step_frequency = step * 15;
        step = 0;
    }
    
    if(INTCONbits.TMR0IF){ //buzzer
        INTCONbits.TMR0IF = 0;
        LATCbits.LATC3 ^= 1;
        if(LATCbits.LATC3 == 1)
        {
            TMR0 = 65535-273;    //set 0.07 sec, (4000000/4)/256 *0.07 = 273 ,  65535-273 = ?
        }
        else if(LATCbits.LATC3 == 0)
        {
            if(buzzerF == 1)    //need to slow
            {
                TMR0 = 65535-1953;       //set 0.5 sec, (4000000/4)/256 *(60/(upper+lower)/8) = 9375 ,  65535-9375 = 56160
            }
            else if(buzzerF == 0)   //need to fast
            {
                TMR0 = 65535-273;       //set 0.07 sec, (4000000/4)/256 *0.2 = 781
            }
        }
    }
    
    return ;
}

void main(void) {
    //setting OSC, 4 MHz
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 0;
    
    sec_10 = 0;
    
    upper = 180;
    lower = 120;
    
    f_one = 1;
    f_two = 0;
    f_three = 0;
    
    count = 0;
    int c_one = 0;
    int c_two = 0;
    int c_three = 0;
    adc_init();
    ccp2_init();
    tmr3_init(0);
    
    TRISBbits.RB1 = 0;      //output
    TRISBbits.RB2 = 0;
    TRISBbits.RB3 = 0;
    
    TRISCbits.RC4 = 0;      //output
    TRISCbits.RC5 = 0;
    TRISCbits.RC6 = 0;
    
    TRISAbits.RA3 = 0;      //output
    TRISAbits.RA4 = 0;      
    TRISAbits.RA5 = 0;
    
    LATBbits.LATB1 = 0;
    LATBbits.LATB2 = 0;
    LATBbits.LATB3 = 0;
    LATCbits.LATC4 = 0;
    LATCbits.LATC5 = 0;
    LATCbits.LATC6 = 0;
    LATAbits.LATA3 = 0;
    LATAbits.LATA4 = 0;
    LATAbits.LATA5 = 0;
    
    
    //step frequency LED
    R=0;                //set color
    G=1;
    B=0;
    
    int Time;
    int p = 0;
    TRISCbits.RC7 = 1;           /* define PORTB as Input port*/
    TRISBbits.RB5 = 0;              /* define PORTD as Output port*/
    TRISDbits.RD1 = 0;
    INTCON2bits.RBPU=0;     /*enable PORTB Pull-ups */
    T1CON = 0x80;           /* enable 16-bit TMR1 Register,No pre-scale,
                             * use internal clock,Timer OFF */
    TMR1IF = 0;             /* make Timer1 Overflow Flag to '0' */
    TMR1=0;                 /* load Timer1 with 0 */
    
    count = 120;
    eye = 0;
    initialize_7();
    // set upper bound
    int upper_flag = 1;
    
    //buzzer
    TRISCbits.RC3 = 0;      //buzzer output
    LATCbits.LATC3 = 0;
    flagBuzzer = 0;
    tmr0_init();
    tmr0_onoff(0);
    
    while(upper_flag){
        int mean = (one + two + three)/3;
        if(one > mean && two < mean && three < mean)
            c_one = 1;
        else if(one < mean && two > mean && three < mean && c_one == 1){
            c_one = 0;
            c_two = 1;
        }else if(one < mean && two < mean && three > mean && c_two == 1){
            c_two = 0;
            eye++;
            upper++;
        }
        if(one < mean && two < mean && three > mean)
            c_three = 1;
        else if(one < mean && two > mean && three < mean && c_three == 1){
            c_three = 0;
            c_two = 1;
        }else if(one > mean && two < mean && three < mean && c_two == 1){
            c_two = 0;
            upper--;
        }
        showPace(upper + 1000);
        if(one > 0x80 && two > 0x180 && three > 0x180)
            upper_flag = 0;
        __delay_us(20);
    }
    
    __delay_ms(1000);
    
    // set lower bound
    int lower_flag = 1;
    while(lower_flag){
        int mean = (one + two + three)/3;
        if(one > mean && two < mean && three < mean)
            c_one = 1;
        else if(one < mean && two > mean && three < mean && c_one == 1){
            c_one = 0;
            c_two = 1;
        }else if(one < mean && two < mean && three > mean && c_two == 1){
            c_two = 0;
            eye++;
            lower++;
        }
        if(one < mean && two < mean && three > mean)
            c_three = 1;
        else if(one < mean && two > mean && three < mean && c_three == 1){
            c_three = 0;
            c_two = 1;
        }else if(one > mean && two < mean && three < mean && c_two == 1){
            c_two = 0;
            lower--;
        }
        showPace(lower + 2000);
        if(one > 0x180 && two > 0x180 && three > 0x180)
            lower_flag = 0;
        __delay_us(20);
    }
    tmr3_init(1);
    //step frequency
    int w = 0;
    int w2 = 0;
    step = 0;
    int close = 0;
    step_frequency = 120;
    
    //breath LED
    PWM1_Init(124);
    PWM1_Start();
    PWM1_Duty(0);
    
    while(1){
        
        //RGB LED
        LATBbits.LATB1 = R;
        LATBbits.LATB2 = G;
        LATBbits.LATB3 = B;
        LATCbits.LATC4 = R;
        LATCbits.LATC5 = G;
        LATCbits.LATC6 = B;
        LATAbits.LATA3 = R;
        LATAbits.LATA4 = G;
        LATAbits.LATA5 = B;
        
        p++;
        Trigger_Pulse_10us();               /* transmit at least 10 us pulse to HC-SR04 */
        while(PORTCbits.RC7==0){
            //Trigger_Pulse_10us(); 
            w++;
        }
        /* wait for rising edge at Echo pin of HC-SR04 */
        TMR1=0;                             /* Load Timer1 register with 0 */
        TMR1ON=1;                           /* turn ON Timer1*/
        while(PORTCbits.RC7==1 ){
            w2++;
        }; /* wait for falling edge at Echo pin of HC-SR04*/
        eye++;
        
        Time = TMR1;                        /* copy Time when echo is received from an object */
       
        TMR1ON=0;
        /* turn OFF Timer1 */
        if(Time > 24)
            Distance = ((float)Time/117.00);         /* distance = (velocity x Time)/2 */
        else{
            TMR1=0;
            TMR1ON=0;
        }
        if(Distance < 10 && close == 0){
            step++;
            close = 1;
        }
        else if(Distance > 10 && close == 1){
            close = 0;
        }
        
        showPace(step_frequency+5000);
        
        //check step frequency and change RGB color
        if((step_frequency>=upper || step_frequency<=lower) && first10>=10)
        {
            if(step_frequency>=upper)    //REB RED, buzzer slow
            {
                R=1;
                G=0;
                B=0;
                buzzerF = 1;
                addBreath = 20;
            }
            else if(step_frequency<=lower)   //RGB BLUE, buzzer fast
            {
                R=0;
                G=0;
                B=1;
                buzzerF = 0;
                addBreath = 10;
            }
            if(flagBuzzer == 0) //turn on buzzer
            {
                flagBuzzer = 1;
                tmr0_onoff(1);
            }
            
        }
        else if(step_frequency<upper && step_frequency>lower)     //RGB GREEN, buzzer stop
        {
            R=0;
            G=1;
            B=0;
            addBreath = 15;
            flagBuzzer = 0;
            tmr0_onoff(0);
        }
        
//        t=step_frequency/4;   ///one breath per minute
//        t=60/t;
//        t=t*2.5;
//        addBreath = t;
        if(flagBreath==0)
        {
            setBreath=setBreath+addBreath;
        }
        else if(flagBreath==1)
        {
            setBreath=setBreath-addBreath;
        }
        if(setBreath>500)
        {
            setBreath = 500;
            flagBreath=1;
        }
        else if(setBreath<10)
        {
            flagBreath=0;
        }
        PWM1_Duty(setBreath);
//        PWM1_Duty(500);
        
        
    }
    return;
}

void adc_init(void){
    //datasheet p232 TABLE 19-2
    //Configure the A/D module
    //ADCON0
    //select analog channel
    //set TRIS
    //Turn on A/D module
    //ADCON1 //set refer voltage

    //ADCON2
    //A/D Conversion Clock
    //Acquisition Time
    //left or right justified
    //Configure A/D interrupt(optional)
    //enable Interrupt Priority mode
    
    //setting OSC, 4 MHz
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 0;
    OSCCONbits.IRCF0 = 1;
    
    ADCON1bits.VCFG1 = 0; //vss
    ADCON1bits.VCFG0 = 0;  //vdd
    ADCON1bits.PCFG = 0b1100;//0111 analog0~7 digital AN8~12
    ADCON0bits.CHS = 0b0000;//0111 channel7 AN7
    TRISEbits.TRISE2 = 1 ; //re2
    ADCON2bits.ADFM = 1;//right most
    
    //setting conversion time
    ADCON2bits.ACQT = 0b001; //2TAD
    ADCON2bits.ADCS = 0b111; //FRC
    ADCON0bits.ADON = 1;//on  ad converter enable
    //setting adc interrupt
    PIE1bits.ADIE = 1;
    IPR1bits.ADIP = 1;
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    RCONbits.IPEN = 1;
    
}

void ccp2_init(void){
    //Configure CCP2 mode,ref datasheet p139
    //configure CCP2 interrupt
    //configure CCP2 comparator value
    CCP2CONbits.CCP2M3 = 1; //1011 compare mode
    CCP2CONbits.CCP2M2 = 0;
    CCP2CONbits.CCP2M1 = 1;
    CCP2CONbits.CCP2M0 = 1;
    CCPR2 = 31250;
    PIE2bits.CCP2IE = 1;  //CCP2 interrupt enable bit
    IPR2bits.CCP2IP = 1;    //priority bit
}

void tmr3_init(int mode){
    
    if(mode == 0){  //for setting
        tmr3Mode = 0;
        //set up timer3, ref datasheet p135
        //no need to turn up timer3 interrupt
        T3CONbits.TMR3ON = 1; //enable timer3
        T3CONbits.T3CCP2 = 1; //1x timer3 is capture /cpmompare clock source for the ccp2
        T3CONbits.T3CCP1 = 1;
        T3CONbits.RD16 = 1;  //enable 16 bit read/write mode on
        PIE2bits.TMR3IE = 0;
    }
    else if(mode == 1){     //for running
        //reset
        T3CON = 0;
        PIE1bits.ADIE = 0;  //close SDC interrupt
        tmr3Mode = 1;
        
        T3CONbits.RD16 = 1;
        
        T3CONbits.T3CKPS1 = 1;	//set Prescaler 1:8
        T3CONbits.T3CKPS0 = 1;
        
        T3CONbits.TMR3CS = 0;   //set clock source
        T3CONbits.TMR3ON = 1;	//Timer1 On


        //set interrupt for Timer1
        IPR2bits.TMR3IP = 1;	//set Timer1 as high interrupt
        PIR2bits.TMR3IF = 0;	//clear Timer1 interrupt flag bit, need to reclear after interrupt
        PIE2bits.TMR3IE = 1;	//enable Timer1 interrupt
        
        TMR3 = 3035;   //0.5 sec
        count2sec = 0;
    }
        
    
}

void Trigger_Pulse_10us()
{
    LATB5= 1;
    __delay_us(10);
    LATB5 = 0;
}

void tmr0_init(void){   //set up timer1
    
    //select 16 bit TMR0
    T0CONbits.T08BIT = 0;
    
    //select use prescaler
    T0CONbits.PSA = 0;
    
    //Timer0 Prescaler 1:256
    T0CONbits.T0PS2 = 1;
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS0 = 1;
    
    //Timer0 clock source
    T0CONbits.T0CS = 0;
    
    //enable Timer0
    //T0CONbits.TMR0ON = 1;
    
    TMR0 = 65534;           //(4000000/4)/256 *(60/(upper+lower)/8) = 9375 ,  65535-9375 = 56160
    
    IPR1bits.TMR1IP = 1;    //set Timer0 as high interrupt
    INTCONbits.TMR0IF = 0;	//clear Timer0 interrupt flag bit, need to reclear after interrupt
    INTCONbits.TMR0IE = 1;	//enable Timer0 interrupt
    
    RCONbits.IPEN = 1;      //enable interrupt priority
    INTCONbits.GIE = 1;     //enable all High priority interrupt
}
void tmr0_onoff(int o)
{
    if(o==1){   //turn on
        T0CONbits.TMR0ON = 1;
        TMR0 = 65534;       //directly interrupt
    }
    else if(o==0){  //turn off
        LATCbits.LATC3 = 0;
        T0CONbits.TMR0ON = 0;
    }
}

PWM1_Init(long setDuty){
    PR2 = setDuty;
    //PR2 = 2ms
    // ( (2*0.001) / 16 * 4000000 / 4 )-1 = 124
}
PWM1_Duty(unsigned int duty){
    //set duty to CCPR1L , CCP1X(CCP1CON:5) and CCP1Y(CCP1CON:4)
    int tempDuty = duty;
    tempDuty = tempDuty<<30;
    tempDuty = tempDuty>>30;
    tempDuty = tempDuty>>1;
    CCP1CONbits.CCP1X = tempDuty;
    tempDuty = duty;
    tempDuty = tempDuty<<31;
    tempDuty = tempDuty>>31;
    CCP1CONbits.CCP1Y = tempDuty;
    CCPR1L = duty >> 2;
    
}
PWM1_Start(){
    //set CCP1CON, mode PWM
    CCP1CONbits.CCP1M0 = 0;
    CCP1CONbits.CCP1M1 = 0;
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M3 = 1;
    
    //set TIMER2 on
    T2CONbits.TMR2ON = 1;
    
    //TRISC: set RC2 as output
    TRISCbits.RC2 = 0;
    
    //set TIMER2 prescaler 1:16
    T2CONbits.T2CKPS0 = 0;
    T2CONbits.T2CKPS1 = 1;
}