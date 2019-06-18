/*
 * RobotCompetition.c
 *
 * Created: 12.06.2019 08:55:16
 * Author : Fabian, Johannes, Kayvan
 */

#include <avr/io.h>

//##################################################################################
//################################ Drive Module ####################################
//##################################################################################

static int prescaler=8;
static int counterValue_PWM=255;

void initPWM(int prescaler_value, int PWMbit_value){			//(checked) running in application
	//setup of PWM parameters ; Prescaler: 1,8,64,256,1024 possible
	//PWMbit: 255,511,1023 possible //255: 8 Bit-PWM , 511: 9 Bit-PWM , 1023: 10 Bit-PWM
	//set PWMbit to PWM counter value you want to use (255 [8bit-counter], 511 [9bit-counter], 1023 [10bit-counter] possible)

	TCCR1A &= ~((1<<COM1A0)|(1<<COM1A1)|(1<<COM1B0)|(1<<COM1B1));
	TCCR1B &= ~((1<<WGM13)|(1<<ICES1)|(1<<ICNC1));

	if(PWMbit_value==255){
		TCCR1A &= ~((1<<WGM11));
		TCCR1B &= ~((1<<WGM13));
		TCCR1A |=(1<<WGM10);
		TCCR1B |=(1<<WGM12);
	}

	if(PWMbit_value==511){
		TCCR1A &= ~((1<<WGM10));
		TCCR1B &= ~((1<<WGM13));
		TCCR1A |=(1<<WGM11);
		TCCR1B |=(1<<WGM12);
	}

	if(PWMbit_value==1023){
		TCCR1B &= ~((1<<WGM13));
		TCCR1A |=(1<<WGM10)|(1<<WGM11);
		TCCR1B |=(1<<WGM12);
	}

	if(prescaler_value==1){
		TCCR1B |= (1<<CS10);
	}

	if(prescaler_value==8){
		TCCR1B |= (1<<CS11);
	}

	if(prescaler_value==64){
		TCCR1B |= (1<<CS10)|(1<<CS11);
	}

	if(prescaler_value==256){
		TCCR1B |= (1<<CS12);
	}

	if(prescaler_value==1024){
		TCCR1B |= (1<<CS10)|(1<<CS12);
	}
}

//################################ Left Wheel ######################################

void LeftWheel(int forward, float dutycycle_forward, float dutycycle_backward,int counterValue_PWM) {		//(checked), running in application
	// Left engine (wheel) control -> forward=1 - wheel turns forward ; forward=0 - wheel turns backward
	// velocity control with PWM -> dutycycle= 0.1-0.9 (1 -> no PWM (full speed))
	// engine turns forward and dutycycle 0.1-0.9 -> fast forward: dutycycle high but not 1; slow forward: dutycycle low but not 0  ; 1-PWM disabled
	// engine turns backward and dutycycle 0.1-0.9 -> fast reverse: dutycycle high ; slow reverse: dutycycle lower but not 0 ; fastest reverse: dutycycle=1 and forward=0 (PWM disabled)
	// set counterValue_PWM to actual used PWM counter value (255 [8bit counter], 511 [9bit counter], 1023 [10bit counter])

	int register_value_forward=0;
	int register_value_backward=0;
	float temp1=0;
	dutycycle_backward= 1-dutycycle_backward;
	temp1= counterValue_PWM*dutycycle_forward;
	register_value_forward= (int)(temp1+0.5);
	temp1= counterValue_PWM*dutycycle_backward;
	register_value_backward= (int)(temp1+0.5);


	if (dutycycle_backward==0.0 && forward==0){
		TCCR1A &= ~((1<<COM1B0)|(1<<COM1B1)); 	//PWM disabled
		PORTD &= ~((1<<PD6));
		PORTB |= (1<<PB2);	//engine backward
	}

	if(dutycycle_forward==1.0 && forward==1){
		TCCR1A &= ~((1<<COM1B0)|(1<<COM1B1));
		PORTB &= ~((1<<PB2));
		PORTD |= (1<<PD6);		//engine forward
	}

	if((dutycycle_backward<1.0 && dutycycle_backward>0) && forward==0){
		OCR1B = register_value_backward; 	//write value in compare register
		TCCR1A |= (1<<COM1B1)|(1<<COM1B0);		//enable inverting PWM on Pin PB2
		PORTD &= ~((1<<PD6));
		PORTB |= (1<<PB2);	//engine backward slower

	}


	if((dutycycle_forward<1.0 && dutycycle_forward>0) && forward==1){
		OCR1B = register_value_forward; 	//write value in compare register
		TCCR1A |= (1<<COM1B1)|(1<<COM1B0); 		//enable inverting PWM on Pin PB2
		PORTB |= (1<<PB2);
		PORTD |= (1<<PD6);	//engine forward slower
	}

}

//################################ Right Wheel #####################################

void RightWheel(int forward, float dutycycle_forward, float dutycycle_backward, int counterValue_PWM) {			//(checked) running in application
	// Right engine (wheel) control -> forward=1 - wheel turns forward ; forward=0 - wheel turns backward
	// velocity control with PWM -> dutycycle= 0.01 - 0.99 (0 -> no PWM (full speed))
	// engine turns backward and dutycycle 0.1-0,9 -> fast reverse: dutycycle high ; slow reverse: dutycycle lower but not 0 ; fastest reverse: dutycycle=1 and forward=0 (PWM disabled)
	// set counterValue_PWM to actual used PWM counter value (255 [8bit counter], 511 [9bit counter], 1023 [10bit counter])

	int register_value_forward=0;
	int register_value_backward=0;
	float temp1=0;
	dutycycle_backward=1-dutycycle_backward;
	temp1=counterValue_PWM*dutycycle_forward;
	register_value_forward=(int)(temp1+0.5);
	temp1=counterValue_PWM*dutycycle_backward;
	register_value_backward=(int)(temp1+0.5);

	if (dutycycle_backward==0.0 && forward==0){
		TCCR1A &= ~((1<<COM1A0)|(1<<COM1A1)); 	//PWM disabled
		PORTD &= ~((1<<PD7));
		PORTB |= (1<<PB1);	//engine backward
	}

	if(dutycycle_forward==1.0 && forward==1){
		TCCR1A &= ~((1<<COM1A0)|(1<<COM1A1));
		PORTB &= ~((1<<PB1));
		PORTD |= (1<<PD7);		//engine forward
	}

	if((dutycycle_backward<1.0 && dutycycle_backward>0) && forward==0){
		OCR1A = register_value_backward; 	//write value in compare register
		TCCR1A |= (1<<COM1A1)|(1<<COM1A0);		//enable inverting PWM on Pin PB1
		PORTD &= ~((1<<PD7));
		PORTB |= (1<<PB1);	//engine backward slower

	}


	if((dutycycle_forward<1.0 && dutycycle_forward>0) && forward==1){
		OCR1A = register_value_forward; 	//write value in compare register
		TCCR1A |= (1<<COM1A1)|(1<<COM1A0); 		//enable inverting PWM on Pin PB1
		PORTB |= (1<<PB1);
		PORTD |= (1<<PD7);		//engine forward slower
	}

}

//################################ Drive ###########################################

void Drive(float dutycycle_forward, int counterValue_PWM) {			//(checked) running in application
	// both wheels move forward, set counterValue_PWM to actual used PWM counter value (255 [8bit counter], 511 [9bit counter], 1023 [10bit counter])									(c1)


	LeftWheel(1,dutycycle_forward,0,counterValue_PWM), RightWheel(1,dutycycle_forward,0,counterValue_PWM);


}

//################################ Reverse ########################################

void Reverse(float dutycycle_backward, int counterValue_PWM) {		//(checked) running in application
	// both wheels move backward
	// set counterValue_PWM to actual used PWM counter value (255 [8bit counter], 511 [9bit counter], 1023 [10bit counter])								(c1)

	LeftWheel(0,0.8,dutycycle_backward,counterValue_PWM), RightWheel(0,0.8,dutycycle_backward,counterValue_PWM);

}

//################################ Turn ###########################################

void Turn(int right, float dutycycle_forward, int counterValue_PWM) {		//(checked) running in application
	// turn engine right=1 -> right turn ; right=0 -> left turn
	// set counterValue_PWM to actual used PWM counter value (255 [8bit counter], 511 [9bit counter], 1023 [10bit counter]) 	(c1)

	if(right==1){
		LeftWheel(1,dutycycle_forward,1,counterValue_PWM);  //, RightWheel(0,dutycycle_forward,dutycycle_backward); // also possible, works with LeftWheel() either
	}

	if(right==0){
		RightWheel(1,dutycycle_forward,1,counterValue_PWM); //,LeftWheel(0,dutycycle_forward,dutycycle_backward)
	}
}

//################################ Stop ##########################################

void StopEngines(void){
	TCCR1A &= ~((1<<COM1A0)|(1<<COM1A1));
	TCCR1A &= ~((1<<COM1B0)|(1<<COM1B1));
	PORTB &= ~((1<<PB1)|(1<<PB2));
	PORTD &= ~((1<<PD6)|(1<<PD7));
}


//##################################################################################
//################################ Acceleration Module #############################
//##################################################################################

//Reads the acceleration at the moment then calls the reverse/turn function if it has slowed down for whatever reason by hitting something, else drive
int AccelerationRead(void) {

}

void Delay(int time) {

}


//##################################################################################
//################################ Light Sensor Module #############################
//##################################################################################

//Reads what the light values are at the moment, if they are below/above certain thresholds reverse/turn, else drive
int LightRead(void) {

}




//##################################################################################
//################################ Bluetooth Module ################################
//##################################################################################


#define FOSC 8000000							// Clock Speed
#define BAUD 4800
#define MYUBRR FOSC/(16*BAUD) - 1				// calculating UBRR

void USART_Init( unsigned int ubrr)
{
	//####################### Set baud rate ########################################
	UBRRH = (unsigned char)(ubrr>>8);
	UBRRL = (unsigned char)ubrr;

	//####################### Data transfer config  ################################
	UCSRA = 0;									// status byte
	UCSRB = (1<<RXEN)|(1<<TXEN);				// Enable receiver and transmitter
	UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);	// Set frame format: 8data, 2stop bit

	// ################ PIN config ##########################################
	//  DDRD |= (1<<PD4)|(1<<PD5)|(1<<PD6);
	//  DDRB |= (1<<PB4)|(1<<PB5)|(1<<PB1)|(1<<PB2);
}

//################################ Read Analog ################################################################

unsigned char Read_Analog( char Kanal)
{
	char zwischenwert = 0;
	ADMUX = ADMUX | (Kanal); // wählt den entsprechenden Analogen Eingang aus, da Binäre Codierung, entspricht z.B. eine 3 den entsprechenden MUX bits im Register
	ADCSRA = ADCSRA | (1<<ADSC); // starten der Wandlung
	while ((ADCSRA & (1<<ADSC)))	// Wartet auf das Beenden der Wandlung
	{
	}
	zwischenwert= ADCH;			// da eine 8 Bit Wert gefordert und das Register Linksseitig beschrieben wird reicht es das H Register auszulesen
	ADMUX = 0b01100000;			// MUX Register werden wieder zurückgesetzt damit es nicht zu Problemen bei der Auswahl am Anfang dieser Funktion kommen kann
	return zwischenwert;		// gibt den glesenen Analogwert zurück
}
//################################ sending receiving #####################################################################

// ############################## sending ############################################

void USART_Transmit( unsigned char data )
{
	while ( !( UCSRA & (1<<UDRE)) )				// Wait for empty transmit buffer
	;
	UDR = data;									// Put data into buffer, sends the data
}

 // ############################## receiving ############################################

 unsigned char Recive(void)
 {
	 while (!(UCSRA & (1<< RXC)))   // ist das 7te bit im UCSRA gesetzt (RXC Flag) liegen noch ungelesenen Daten im Register
	 {							   // solange dies nicht der Fall ist verbleibt das Programm in der Schleife und wartet
	 }							   // auf die neuen Daten
	 return UDR;					   // gibt die daten aus dem UDR heraus
 }

 // ############################## sending ASCII ############################################

 void Sende_In_ASCII (unsigned char Analogwert)
 {
	 unsigned char umgewandelte_Zahl[3]; // 3 damit noch eine Lücke gesendet wird
	 umgewandelte_Zahl[3]= ' ';
	 unsigned char zahl = Analogwert;

	 // transfer to ASCII
	 umgewandelte_Zahl[2] = (zahl % 10) + 48 ;   // 123 / 10 gibt hier den Rest 3    // +48 da in ASCII die 48 die 0 ist
	 zahl = zahl / 10;					        // ganzzahlige division 123 / 10 = 12
	 umgewandelte_Zahl[1] =( zahl % 10) + 48 ;   // 12 / 10 gibt hier den Rest 2
	 zahl = zahl / 10;							// 12 /10 = 1
	 umgewandelte_Zahl[0] = zahl + 48 ;


	 // sending ASCII letters
	 UCSRB = (1<<TXEN);	// Senden ein Empfangen aus
	 int i = 0;

	 for ( i = 0; i <= 3; i++)
	 {
		 USART_Transmit (umgewandelte_Zahl[i]);
		 while (!(UCSRA & (1<< TXC)))		// Wartet auf Ende der Übertragung
		 {
		 }
		 UCSRA = UCSRA | (1<< TXC);
	 }

 }


//##################################################################################
//################################ Main ############################################
//##################################################################################

int main(void)
{
	//######################## IO config engine ####################################
	DDRB |= (1<<PB1)|(1<<PB2);
	DDRD |= (1<<PD6)|(1<<PD7);
	initPWM(prescaler,counterValue_PWM);
	float dutyc_f=0.32;							//slower forward duty cycle
	//float dutyc_b=0.21;						//slower backward dutycycle

//################## Input and Output Setups rough draft ###########################
	PINC &= ~((1 << PC2) | (1 << PC3) | (1 << PC1)); //
	PIND &= 0;
	PINB &= 0;

    while (1)
    {
		//######################## Example Drive ###################################
		Drive(dutyc_f,counterValue_PWM);
	    	LightRead();
		AccelerationRead();
		Delay(0);
    }

	USART_Init ( MYUBRR );						// Bluetooth Module
}

