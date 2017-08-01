#include "mySerial.h"
#include <iostream>
using namespace std;
#include <stdlib.h>
#include <stdio.h>



int  main(void)
{

    mySerial serial("/dev/ttyUSB1",1998848);

    // One Byte At the time
    // serial.Send(128);
    // serial.Send(132);
    
    // An array of byte
   unsigned char  data[32];
   // serial.Send(dataArray,sizeof(dataArray));
	
    // Or a string
   // serial.Send("this is it\r\n");
	while (1) {
	serial.Receive(data, 32);
	printf("we receive data %d \n",data[0]);
	}

    return 0;
}
