#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


// Definitions //
#define BUFSIZE 1024
#define MIN_VEL 0
#define MAX_VEL 0

// Velocity Structure //
struct Velocity {
	int v;
	int theta;
}

// Global velcity struct
volatile struct Velocity a;
volatile a.v = 0;
volatile a.theta 0;

int main()
{

	char input = '';
	while(1)
	{
		switch(input)
		{
			case 'w' :
				a.v += 2;
				break;
			case 's' :
				a.v -= 2;
				break;
			case 'a' :
				a.theta += 2;
				break;
			case 'd' :
				a.theta -= 2;
				break;
			default :
				break;
		}

		if (a.v < MIN_VEL) a.v = MIN_VEL;
		if (a.v > MAX_VEL) a.v = MAX_VEL;
	}
}