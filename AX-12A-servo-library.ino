#include "Arduino.h"
#include "src/AX12A.h"

#define DirectionPin 	(10u)
#define BaudRate  		(1000000ul)
#define ID				(1u)

void setup()
{
	ax12a.begin(BaudRate, DirectionPin, &Serial);
}

void loop()
{
	ax12a.ledStatus(ID, ON);
	delay(1000);
	ax12a.ledStatus(ID, OFF);
	delay(1000);
}
