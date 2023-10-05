#include <stdio.h>

#define GDK3_RUN 1

unsigned int g_flags = GDK3_RUN;
int main()
{
	do{
		printf("Hello, world!");
		printf("\n");
	}while(g_flags&GDK3_RUN);

	return 0;
}

#ifndef __NO_SYSTEM_INIT
void SystemInit()
{}
#endif
