/*
 * RobotCompetition.c
 *
 * Created: 12.06.2019 08:55:16
 * Author : Fabian, Johannes, Kayvan
 */ 

#include <avr/io.h>

void LeftWheel(int forward) {
	
}

void RightWheel(int forward) {
	
}

void Drive(void) {
	
}

void Reverse() {
	
}

void Turn(int right) {
	
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
{
	//The Input and Output Setups rough draft
	PINC &= ~((1 << PC2) | (1 << PC3) | (1 << PC1));
	PIND &= 0;
	PINB &= 0;
	
	/* Replace with your application code */
    while (1) 
    {
		LightRead();
		AccelerationRead();
		Delay(0);
    }
}

