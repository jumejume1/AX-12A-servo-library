/*
 * Example showing how to use endless mode (wheel mode) on AX-12A
 * Be sure you removed all mechanical assemblies (hinges) before using this code !
 */
#include "Arduino.h"
#include "../../src/AX12A.h"

#define DirectionPin 	(10u)
#define BaudRate  		(1000000ul)
#define ID				(1u)

void setup()
{
	ax12a.begin(BaudRate, DirectionPin, &Serial);
	ax12a.setEndless(ID, ON);
	ax12a.turn(ID, LEFT, 100);
}

void loop()
{

}
