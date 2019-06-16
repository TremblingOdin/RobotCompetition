/*
 * RobotCompetition.c
 *
 * Created: 12.06.2019 08:55:16
 * Author : Fabian, Johannes, Kayvan
 */ 

#include <avr/io.h>

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

void Drive(float dutycycle_forward, int counterValue_PWM) {			//(checked) running in application
	// both wheels move forward, set counterValue_PWM to actual used PWM counter value (255 [8bit counter], 511 [9bit counter], 1023 [10bit counter])									(c1)
	

	LeftWheel(1,dutycycle_forward,0,counterValue_PWM), RightWheel(1,dutycycle_forward,0,counterValue_PWM);
	

}

void Reverse(float dutycycle_backward, int counterValue_PWM) {		//(checked) running in application
	// both wheels move backward
	// set counterValue_PWM to actual used PWM counter value (255 [8bit counter], 511 [9bit counter], 1023 [10bit counter])								(c1)
	
	LeftWheel(0,0.8,dutycycle_backward,counterValue_PWM), RightWheel(0,0.8,dutycycle_backward,counterValue_PWM);
	
}

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

void StopEngines(void){
	TCCR1A &= ~((1<<COM1A0)|(1<<COM1A1));
	TCCR1A &= ~((1<<COM1B0)|(1<<COM1B1));
	PORTB &= ~((1<<PB1)|(1<<PB2));
	PORTD &= ~((1<<PD6)|(1<<PD7));
}


//Reads what the light values are at the moment, if they are below/above certain thresholds reverse/turn, else drive
int LightRead(void) {
	
}

//Reads the acceleration at the moment then calls the reverse/turn function if it has slowed down for whatever reason by hitting something, else drive
int AccelerationRead(void) {
	
}

void Delay(int time) {
	
}

int main(void)
{	//IO config engine
	DDRB |= (1<<PB1)|(1<<PB2);
	DDRD |= (1<<PD6)|(1<<PD7);
	initPWM(prescaler,counterValue_PWM);
	float dutyc_f=0.32;  //slower forward dutycycle
	//float dutyc_b=0.21;  //slower backward dutycycle

	//The Input and Output Setups rough draft
	PINC &= ~((1 << PC2) | (1 << PC3) | (1 << PC1));
	PIND &= 0;
	PINB &= 0;
	
	/* Replace with your application code */
    while (1) 
    {
		Drive(dutyc_f,counterValue_PWM);	//example Drive 
	    	LightRead();
		AccelerationRead();
		Delay(0);
    }
}

