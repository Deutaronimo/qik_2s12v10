#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <thread>
#include <chrono>
#include "qik.hpp"

QIK::QIK()
{
	serialPort = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

	if (serialPort < 0) 
	{
	    std::cout << RED ;
	    std::cerr << "Failed to open port: " << portName << std::endl;
            
        }
        
	std::cout << GREEN ;
        
	std::cout << "Serial port opened: " << std::endl;
	
	if(use8Bit) 
	{
	    std::cout << "Using 8 bit mode." << std::endl;
	}
	
	else
	    std::cout << "Using 7 bit mode." << std::endl;
	
	std::cout << RESET;

	configurePorts();
}


QIK::~QIK()
{
    close(serialPort);
    std::cout << "Closed serial port." << std::endl;
}


void QIK::setDeviceId(uint8_t id)
{
	// Set device ID of controller.
	if(id < 0 | id > 128)
	{
		std::cout << "Error setting device id, Out of range." << std::endl;
		return;
	}

	uint8_t _id = id;
	uint8_t command[4] = {STARTBYTE, DEVICE_ID, command_setDeviceId, _id};
	write(serialPort, command, sizeof(command));
	delay(5);
}


void QIK::motor_m0_Forward(uint8_t speed)
{

	if(speed < 0 | speed > 128)
	{
		std::cout << "Speed is out of range.\n";
		return;
	}

	std::cout << "Motor M0 Forward: " << static_cast<int>(speed) << std::endl;
	uint8_t command[4] = {STARTBYTE, DEVICE_ID, command_m0_forward, speed};
	write(serialPort,command, sizeof(command));
	delay(5);
}


void QIK::motor_m1_Forward(uint8_t speed)
{

	if(speed < 0 | speed > 128)
	{
		std::cout << "Speed is out of range.\n";
		return;
	}
    
	std::cout << "Motor M1 Forward: " << static_cast<int>(speed) << std::endl;
	uint8_t command[4] = {STARTBYTE, DEVICE_ID, command_m1_forward, speed};
	write(serialPort,command, sizeof(command));
	delay(5);
}


void QIK::motor_m0_Reverse(uint8_t speed)
{
	if(speed < 0 | speed > 128)
	{
		std::cout << "Speed is out of range.\n";
		return;
	}

	std::cout << "Motor M0 reverse: " << static_cast<int>(speed) << std::endl;
	uint8_t command[4] = {STARTBYTE, DEVICE_ID, command_m0_reverse, speed};
	write(serialPort,command, sizeof(command));	
	delay(5);
}


void QIK::motor_m1_Reverse(uint8_t speed)
{
	if(speed < 0 | speed > 128)
	{
		std::cout << "Speed is out of range.\n";
		return;
	}

	std::cout << "Motor M1 reverse: " << static_cast<int>(speed) << std::endl;
	uint8_t command[4] = {STARTBYTE, DEVICE_ID, command_m1_reverse, speed};
	write(serialPort,command, sizeof(command));	
	delay(5);
}


void QIK::motor_m0_Brake(uint8_t value)
{
	if(value< 0 | value > 128)
	{
		std::cout << "Value is out of range.\n";
		return;
	}

	std::cout << "Motor M0 brake: " << static_cast<int>(value) << std::endl;
	uint8_t command[4] = {STARTBYTE, DEVICE_ID, command_m0_setBrake, value};
	write(serialPort,command, sizeof(command));	
	delay(5);
}


void QIK::motor_m1_Brake(uint8_t value)
{
	if(value< 0 | value > 128)
	{
		std::cout << "Value is out of range.\n";
		return;
	}

	std::cout << "Motor M1 brake: " << static_cast<int>(value) << std::endl;
	uint8_t command[4] = {STARTBYTE, DEVICE_ID, command_m1_setBrake, value};
	write(serialPort,command, sizeof(command));	
	delay(5);
}


void QIK::getMotor_m0_current()
{
	uint8_t command[3] = {STARTBYTE, DEVICE_ID, command_m0_getCurrent};
	
	write(serialPort, command, sizeof(command));

    usleep(10000); // Wait for response

    unsigned char response[1];
    
    uint8_t bytesRead = read(serialPort, response, 1);

    if (bytesRead > 0)
	{
		std::cout << "M0 Current reading: " << static_cast<int>(response[0]) << std::endl;
	}

	else
		std::cout << "No current data returned on M0." << std::endl;

	delay(5);
}


void QIK::getMotor_m1_current()
{
	uint8_t command[3] = {STARTBYTE, DEVICE_ID, command_m1_getCurrent};
	
	write(serialPort, command, sizeof(command));

    usleep(10000); // Wait for response

    unsigned char response[1];
    
    uint8_t bytesRead = read(serialPort, response, 1);

    if (bytesRead > 0)
	{
		std::cout << "M1 Current reading: " << static_cast<int>(response[0]) << std::endl;
	}

	else
		std::cout << "No current data returned on M1." << std::endl;
	delay(5);
}


void QIK::getMotor_m0_speed()
{
	uint8_t command[3] = {STARTBYTE, DEVICE_ID, command_m0_getSpeed};
	
	write(serialPort, command, sizeof(command));

    usleep(10000); // Wait for response

    unsigned char response[1];
    
    uint8_t bytesRead = read(serialPort, response, 1);

    if (bytesRead > 0)
	{
		std::cout << "M0 Speed reading: " << static_cast<int>(response[0]) << std::endl;
	}

	else
		std::cout << "No speed data returned on M0." << std::endl;

	delay(5);
}


void QIK::getMotor_m1_speed()
{
	uint8_t command[3] = {STARTBYTE, DEVICE_ID, command_m1_getSpeed};
	
	write(serialPort, command, sizeof(command));

    usleep(10000); // Wait for response

    unsigned char response[1];
    
    uint8_t bytesRead = read(serialPort, response, 1);

    if (bytesRead > 0)
	{
		std::cout << "M1 Speed reading: " << static_cast<int>(response[0]) << std::endl;
	}

	else
		std::cout << "No speed data returned on M1." << std::endl;
	
	delay(5);
}


void QIK::configurePorts()
{
    // Configure port
    tcgetattr(serialPort, &options);

    cfsetispeed(&options, B9600); // Qik default baud rate
    cfsetospeed(&options, B9600);

    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // One stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;

    tcflush(serialPort, TCIFLUSH);
    tcsetattr(serialPort, TCSANOW, &options);

}


void QIK::getErrors()
{
    
	uint8_t command[3] = {STARTBYTE, DEVICE_ID, command_getErrors};
	write(serialPort, command, sizeof(command));

    usleep(10000); // Wait for response

    unsigned char response[1];
    
    uint8_t bytesRead = read(serialPort, response, 1);

    if (bytesRead > 0) 
    {
        std::cout << "Error Byte: " << static_cast<int>(response[0]) << std::endl;
        
	if(response[0] & ERROR_MOTOR_0 ){std::cout << "Motor 0 Fault\n";}

	if(response[0] & ERROR_MOTOR_1)	{std::cout << "Motor 1 Fault\n";}

	if(response[0] & ERROR_MOTOR_0_OVERCURRENT){std::cout << "Motor 0 Overcurrent\n";}

	if(response[0] & ERROR_MOTOR_1_OVERCURRENT){std::cout << "Motor 1 Overcurrent\n";}

	if(response[0] & ERROR_SERIAL_HARDWARE){std::cout << "Serial Hardware Error\n";}

	if(response[0] & ERROR_CRC){std::cout << "CRC Error\n";}

	if(response[0] & ERROR_FORMAT){std::cout << "Format Error\n";}

	if(response[0] & ERROR_TIMEOUT){std::cout << "Timeout\n";}

    } 
    else 
    {
        std::cerr << "No response received." << std::endl;
    }

	delay(5);

}


void QIK::getFirmwareVersion()
{

    uint8_t command[3] = {STARTBYTE, DEVICE_ID, command_getFirmwareVersion};
	write(serialPort, command, sizeof(command));
    
	usleep(10000);
    unsigned char response[1];
    uint8_t bytesRead = read(serialPort, response,1);
    
    if(bytesRead > 0)
    {
	    std::cout << "Firmware Version: " << response[0] << std::endl;
    }

	delay(5);

}


void QIK::getParameter(uint8_t command)
{
	delay(5);
}


void QIK::setPWM(uint8_t value)
{
	delay(5);
}


void QIK::setMotorShutdownOnError(uint8_t value)
{
	delay(5);
}


void QIK::setSerialTimeout(uint8_t value)
{
	delay(5);
}


void QIK::setMotorAcceleration(uint8_t value)
{
	delay(5);
}


void QIK::setBrakeDuration(uint8_t value)
{
	delay(5);
}


void QIK::setMotorCurrentLimit(uint8_t value)
{
	delay(5);
}


void QIK::setMotorCurrentResponse(uint8_t value)
{
	delay(5);
}


void delay(int duration)
{
    std::cout << "Delay: " << duration << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));
}

