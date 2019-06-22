/*
 * RobotCompetition.c
 *
 * Created: 12.06.2019 08:55:16
 * Author : Fabian, Johannes, Kayvan
 */

#include <avr/io.h>

#define READ 1
#define WRITE 0
#define SAD 0b00110000

#define STATUS_REG 0b01001110
#define OUT_X_L 0b01010000
#define OUT_X_H 0b01010010
#define OUT_Y_L 0b01010100
#define OUT_Y_H 0b01010110

#define FOSC 8000000							// Clock Speed
#define BAUD 4800


// Definitions are in the Modules for better understanding
// if they need to be on top, you can change it

static int prescaler = 8;
static int counterValue_PWM = 255;

static char baseX = 0;
static char ongoingX = 0;
static char ongoingY = 0;

static float dutyc_f = 0.5;
static float dutyc_b = 0.5;

//Which side is the bot starting on?
const int side = 0;

//General Purpose Function
void Delay(int time) {
	int i,j;
	for(j = 0;  j < time;  j++){
		for(i = 0; i < 5000; i++);
	}
}


//##################################################################################
//################################ Bluetooth Module ################################
//##################################################################################


void USART_Init( unsigned int ubrr)
{
	//####################### Set baud rate ########################################
	UBRRH = (unsigned char)(ubrr>>8);	// set baud rate
	UBRRL = (unsigned char)ubrr;

	//####################### Data transfer config  ################################
	UCSRA = 0;	// status byte
	UCSRB = (1<<RXEN)|(1<<TXEN);	// Enable receiver and transmitter
	UCSRC = (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);	// Set frame format: 8data, 1stop bit
}

// ############################## sending ##########################################

void USART_Transmit( unsigned char data )
{
	while ( !( UCSRA & (1<<UDRE)) );	// Wait for empty transmit buffer
	UDR = data;	// Put data into buffer, sends the data
	while (!(UCSRA & (1<< TXC)))	// Wartet auf Ende der Übertragung
	{
	}
	UCSRA = UCSRA | (1<< TXC);
}

// ######################### receiving ############################################

unsigned char Recive(void)
{
	while (!(UCSRA & (1<< RXC)))
	{
	}
	return UDR;
}

// ####################### sending ASCII ##########################################

void Sende_In_ASCII (unsigned char Analogwert)
{
	unsigned char value_Array[3];	// 3 damit noch eine Lücke gesendet wird

	unsigned char hundert = Analogwert/100;
	Analogwert -= hundert*100;

	unsigned char zehner = Analogwert/10;
	Analogwert -= zehner*10;

	unsigned char einser = Analogwert;

	value_Array[0]= (hundert+48);
	value_Array[1]= (zehner+48);
	value_Array[2]= (einser+48);

	// sending ASCII letters

	int i = 0;

	for ( i = 0; i < 3; i++)
	{
		USART_Transmit (value_Array[i]);
		i++;
	}

}

//##################################################################################
//################################ Drive Module ####################################
//##################################################################################

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

//TWCR TWINT TWEN and TWSTA might not be the right variable  names, need  to double check
int Acknowledge(void) {
	return !(TWCR & (1 << TWINT));
}

//Sends Start Condition to I2C
void SendStart(int repeated) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (Acknowledge());
	
	USART_Transmit((TWSR & 0xF8));	
}

void AccelerometerRegisterRequest(char regAddress, int read) {
	regAddress |= (read);
	TWDR = regAddress;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (Acknowledge());
	if(read) {
		
	} else {
		
	}
	USART_Transmit(TWSR & 0xF8);
}

void SendSubAddress(char subAddress) {
	//SubAddresses do not need the read/write LSB
	//A 7 bit address is expected so shifting the whole address to the right is necessary
	subAddress = (subAddress >> 1);
	TWDR = subAddress;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (Acknowledge());
	
	USART_Transmit(TWSR & 0xF8);
}

void SendStop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	while(TWCR & (1<<TWINT));
	TWCR &= ~(1<<TWSTO);
	
	USART_Transmit(TWSR & 0xF8);
}

char ReadRequest(char readReg) {
	//USART_Transmit('A');
	SendStart(0);
	//USART_Transmit('B');
	AccelerometerRegisterRequest(SAD, WRITE);
	//USART_Transmit('C');
	SendSubAddress(readReg);
	//USART_Transmit('D');
	SendStart(1);
	//USART_Transmit('E');
	AccelerometerRegisterRequest(SAD, READ);
	//USART_Transmit('F');
	SendStop();

	return TWDR;
}

//Reads and returns the slope of the provided axis
char FindSlope(char axis) {
	char x, y;
	if (axis == 'x') {
		x = ReadRequest(OUT_X_H);
		
		return x;
	}
	else if (axis == 'y') {
		y = ReadRequest(OUT_Y_H);

		return y;
	}
	else {
		return 0;
	}
}

//Checks Status of the X or Y acceleration
void CheckStop(void) {
	char xh, yh;

	xh = ReadRequest(OUT_X_H);

	yh = ReadRequest(OUT_Y_H);

	if (xh < ongoingX - 15) {
		Reverse(dutyc_b, counterValue_PWM);
		if (yh < 0) {
			Turn(0, dutyc_f, counterValue_PWM);
		}
		else {
			Turn(1, dutyc_f, counterValue_PWM);
		}
	}

	if (yh < ongoingY - 15 || yh > ongoingY + 15) {
		Reverse(dutyc_b, counterValue_PWM);
		if (yh < 0) {
			Turn(0, dutyc_f, counterValue_PWM);
		}
		else {
			Turn(1, dutyc_f, counterValue_PWM);
		}
	}
}

void AccelerationRead() {
	char status;

	status = ReadRequest(STATUS_REG);
			
	//If there is a difference in the acceleration of Y or X check if stopped/bumped
	if (!((status & 0b00000011) == 0)) {
		CheckStop();
	}

	//If Y returns a change, find the y slope
	if (!((status & 0b00000010) == 0)) {
		ongoingY = FindSlope('y');
	}

	//If X returns a change find the X slope and the direction in Y we need to change
	if (!((status & 0b00000001) == 0)) {
		ongoingX = FindSlope('x');
		//Is X negative?
		if ((ongoingX & 0b10000000) == 0 && ongoingX > 0b00000000) {
			//Is Y negative? turn right
			if ((ongoingY & 0b10000000) != 0) {
				Turn(1, dutyc_f, counterValue_PWM);
			}
			//Is X negative turn left
			else {
				Turn(0, dutyc_f, counterValue_PWM);
			}
			//Is X going downhill?
		} else if (ongoingX < 0b0000000) {
			Turn(side, dutyc_f, counterValue_PWM);
		}
	}
}

//##################################################################################
//################################ Light Sensor Module #############################
//##################################################################################

//Reads what the light values are at the moment, if they are below/above certain thresholds reverse/turn, else drive


/*

ADMUX = (1<<REFS0)|(1<<ADLAR);				// init analog V_cc with capacitor at AREF-Pin
ADCSRA= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);	// enable A/D-converter


ADMUX |= (1<<MUX1);							// ADC2 active

PORTD |= (1<<PC2);							// light right
PORTD |= (1<<PC3);							// light left



int LightRead(void) {

	unsigned char zw=0;
	ADCSRA = ADCSRA | (1<<ADSC);

	while((ADCSRA & (1<<ADSC)))				// wait for converter end
	{

	}
	zw=ADCH;


	if(zw>7)
	{
		Drive(1,1);
		char_ASCII(zw);
	}


	if(zw<=7)
	{
		Reverse(1,1);
		long back=5000000;
		while(back>=0)
		{
			back--;
		}

		Turn(0,1,0);
		long wert=1000000;
		while(wert>=0)
		{
			wert--;
		}
		else

}

*/


//##################################################################################
//################################ Main ############################################
//##################################################################################

int main(void)
{
	DDRB |= (1<<PB1)|(1<<PB2);
	DDRD |= (1<<PD6)|(1<<PD7);

	DDRD &= ~(1<<PD0);
	DDRD |= (1<<PD1);

	USART_Init (103);
	unsigned char test_value='A';


	initPWM(prescaler,counterValue_PWM);

	//######################## Accelerometer config ################################
	PIND |= (1 << PD0) | (1 << PD1);
	PINC |= (1 << PC4) | (1 << PC5);
	TWBR = 100;

	USART_Transmit(ReadRequest(0x0F));
	baseX = FindSlope('x');

	while (1)
	{
		//######################## Example Drive ###################################

		
		//Drive(dutyc_f,counterValue_PWM);

		//LightRead();

		AccelerationRead();

		//USART_Transmit(test_value);

		//for(volatile long t=100000;t>0;t--);

	}
}