#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>
#include "qik.hpp"

// A quick demo of the QIK serial driver. No delayse are needed between serial commands
// because there is a 5mS delay after each command function.

int main()
{
    QIK qik;
    
    qik.motor_m0_Forward(32);
    qik.motor_m1_Forward(32);

    delay(2000);  

    qik.motor_m0_Reverse(32);
    qik.motor_m1_Reverse(32);
    
    delay(2000);

    qik.motor_m0_Forward(0);
    qik.motor_m1_Forward(0);
       	
    return 0;
}

