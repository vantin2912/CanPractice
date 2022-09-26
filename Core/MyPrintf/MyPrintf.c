/*
 * MyPrintf.c
 *
 *  Created on: Sep 23, 2022
 *      Author: Kino Van Tin
 */


#include "MyPrintf.h"
#include <stdarg.h>


osMutexId_t PrintfMutex;

const osMutexAttr_t PrintfMutex_attr = {
  "PrintfMutex",     // human readable mutex name
  osMutexRecursive | osMutexPrioInherit,    // attr_bits
  NULL,                // memory for control block
  0U                   // size for control block
};

void 	MyPrintf_Init()
{
	PrintfMutex = osMutexNew(&PrintfMutex_attr);
}

int	MyPrintf (const char *__restrict format, ...)
{
	osMutexAcquire(PrintfMutex, 0);

	va_list arg;
	char Buffer[PrintBufferSize]={0};
	size_t size=sizeof(Buffer);
	va_start(arg,format);
	char* Output  = vasnprintf(Buffer,&size,format,arg);
	va_end(arg);
	if(Output != NULL)
	{
		HAL_UART_Transmit(&DebugUART,(uint8_t*) Output, size, 10);

		if(Output != Buffer)
		{

			vPortFree(Output);
		}
	}
	osMutexRelease(PrintfMutex);

	return size;
}
