#pragma once
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>


#define RESET  "\033[0m"
#define BOLD   "\033[1m"
#define RED    "\033[31m"
#define GREEN  "\033[32m"
#define YELLOW "\033[33m"
#define BLUE   "\033[34m"
#define CYAN   "\033[36m"

#define MOTOR_0 0
#define MOTOR_1 1

#define STARTBYTE 0xAA
#define DEVICE_ID 0x0A

class QIK
{
    termios options;
    
    std::string CLEAR = "\033[2J\033[1;1H";

    bool use8Bit = true;
    
    // Bit flags in error byte.
    const uint8_t ERROR_MOTOR_0 = 1 << 0;
    const uint8_t ERROR_MOTOR_1 = 1 << 1;
    const uint8_t ERROR_MOTOR_0_OVERCURRENT = 1 << 2;
    const uint8_t ERROR_MOTOR_1_OVERCURRENT = 1 << 3;
    const uint8_t ERROR_SERIAL_HARDWARE     = 1 << 4;
    const uint8_t ERROR_CRC     = 1 << 5;
    const uint8_t ERROR_FORMAT  = 1 << 6;
    const uint8_t ERROR_TIMEOUT = 1 << 7;

    const char* portName = "/dev/ttyS0";
    unsigned char deviceID  = 0x0A;
    const unsigned char startByte = 0xAA;
    
    int serialPort = 0;  

    // This is a list of the Pololu Protocol commands.
    uint8_t command_setDeviceId        = 0x00;
    uint8_t command_getFirmwareVersion = 0x01;
    uint8_t command_getErrors          = 0x02;
    uint8_t command_getParameter       = 0x03;
    uint8_t command_setParameter       = 0x04;
    
    uint8_t command_m0_setBrake        = 0x06;
    uint8_t command_m1_setBrake        = 0x07;

    uint8_t command_m0_forward         = 0x08;
    uint8_t command_m0_forward_8bit    = 0x09;
    uint8_t command_m0_reverse         = 0x0A;
    uint8_t command_m0_reverse_8bit    = 0x0B;

    uint8_t command_m1_forward         = 0x0C;
    uint8_t command_m1_forward_8bit    = 0x0D;
    uint8_t command_m1_reverse         = 0x0E;
    uint8_t command_m1_reverse_8bit    = 0x0F;

    uint8_t command_m0_getCurrent      = 0x10;
    uint8_t command_m1_getCurrent      = 0x11;

    uint8_t command_m0_getSpeed        = 0x12;
    uint8_t command_m1_getSpeed        = 0x13;

	public:

    QIK();
    ~QIK();
    
    void configurePorts(); //
    
    void motor_m0_Forward(uint8_t speed); //
    void motor_m1_Forward(uint8_t speed); //

    void motor_m0_Reverse(uint8_t speed); //
    void motor_m1_Reverse(uint8_t speed); //

    void motor_m0_Brake(uint8_t ammount); //
    void motor_m1_Brake(uint8_t ammount); //

    void getErrors(); //
    void getFirmwareVersion(); //
    void getParameter(uint8_t command);

    void getMotor_m0_current(); //
    void getMotor_m1_current();  //

    void getMotor_m0_speed(); //
    void getMotor_m1_speed(); //
    
    void setDeviceId(uint8_t ID); //   

    void setPWM(uint8_t value);
    void setMotorShutdownOnError(uint8_t value);
    void setSerialTimeout(uint8_t value);
    void setMotorAcceleration(uint8_t value);
    void setBrakeDuration(uint8_t value);
    void setMotorCurrentLimit(uint8_t value);
    void setMotorCurrentResponse(uint8_t value);


};

void delay(int duration);
