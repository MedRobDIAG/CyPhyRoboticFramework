#include "HapticDevice.h"

HapticDevice::HapticDevice()
{
    frequency = 0;
    is_connected = false;
    interval = 1000;
    motorCounter = 0;
    sendData = 0;

    running = true;
    
    #ifndef _WIN32 
		identifier = -1;
        //pthread_mutex_init(&posMutex, NULL);
    #endif
}

void HapticDevice::stop()
{
    running = false;
    this->sendSignal(0);
}

void HapticDevice::sendSignal_all(const Eigen::Vector4i& motors)
{
#ifdef _WIN32 
    //char buffer[12] = { (char)0x53 , (char)0x61 , (char)0x61 , (char)0x61 , (char)0x61 , (char)0x45 };
	char buffer[12] = { (char)0x53 , (char)motors(0) , (char)motors(1) , (char)motors(2) , (char)motors(3) , (char)0x45 };
	/*buffer[0] = 0x53;
    buffer[1] = 0x61;
    buffer[2] = 0x61;
    buffer[3] = 0x61;
    buffer[4] = 0x61;
    buffer[5] = 0x45;*/
    //sprintf(&buffer[0], "%c%c%c%c%c%c", 0x53, 0x20, 0x20, 0x20, 0x20, 0x45);
    //char* buffer = value;
    //sprintf(&buffer[0], "%c%c%c%c%c%c", 0x53, 0x20, 0x20, 0x20, 0x20, 0x45);
    //sprintf(&buffer[0], "%c%c%c%c%c%c", 0x53, 0x61, 0x61, 061, 0x61, 0x45);
    //sprintf(buffer, "%c", value);
    //buffer = value;
    if (!this->identifier->WriteData(buffer, 6))
    {
        printf("Error in sending data\n");
    }
#else
    int buffer[1] = { value };
    for (int i = 0; i < 1; i++)
        serialport_writebyte(this->identifier, buffer[i]);
#endif
}

void HapticDevice::sendSignal(int value)
{
    #ifdef _WIN32 
        char buffer [2];
 
        sprintf(buffer, "%c", value);
        if(!this->identifier->WriteData(buffer, 1))
        {
            printf("Error in sending data\n");
        }
    #else
        int buffer[1] = {value};
        for(int i = 0; i < 1; i++)
            serialport_writebyte(this->identifier, buffer[i]);
    #endif
}

void HapticDevice::stopMotors()
{
    #ifdef _WIN32 
        char buffer[1];
        //sprintf(&buffer[0],"%c",0);
        sprintf(&buffer[0], "%c%c%c%c%c%c", 0x53, 0x20, 0x20, 0x20, 0x20, 0x45);
        //if(!this->identifier->WriteData(buffer,1))
        if (!this->identifier->WriteData(buffer, 6))
        {
            printf("Error in sending data\n");
        }
    #else
        for(int i = 0; i < 1; i++)
            serialport_writebyte(this->identifier, 0);
    #endif
}

#ifdef _WIN32 
    void HapticDevice::startCommunication(const wchar_t* serialport)
#else
    void HapticDevice::startCommunication(const char* serialport)
#endif
{
        // Baudrate fixed at 9600
    #ifdef _WIN32 
        Serial* serialTmp = new Serial(serialport);
        this->identifier = serialTmp;
        if(this->identifier->IsConnected())
            this->is_connected = true;
    #else
        this->identifier = serialport_init(serialport);
        if (this->identifier >= 0)
            this->is_connected = true;
    #endif
}

void HapticDevice::closeCommunication()
{
    this->is_connected = false;
    
    #ifdef _WIN32 
        this->identifier->~Serial();
    #else
        serialport_close(this->identifier);
    #endif
}

void HapticDevice::run(const Eigen::Vector4i& motors_PWM)
{
    motorCounter = 0;
    //timer.start(); // start time
    int pwm_flat = 20;
    int dir = -1;
    sendSignal_all(motors_PWM);

    // while(this->running)
    // {
    //     // milliseconds
    //     //elapsedTime = timer.elapsed(); //timer.nsecsElapsed();
    //     //felapsedTime = (double)(elapsedTime/1000.0); // convert to float

    //     //qDebug() << this->sendData;

    //     // timer
	// 	std::cout << "Trying to send data" << std::endl;
    //     //int motors_PWM[4] = {0x61,0x61,0x61,0x61};

    //     sendSignal_all(motors_PWM);
    //     if (pwm_flat <= 20)
    //     {
    //         dir = 1;
    //     }
    //     if(pwm_flat >= 255)
    //     { 
    //         dir = -1;
    //     }
    //     //dir=pwm_flat <= 20 ? 1: pwm_flat >= 255 ? -1:1;
    //     pwm_flat += dir;
    //     //motors_PWM[0] = motors_PWM[1] = motors_PWM[2] = motors_PWM[3] = pwm_flat;
    //     motors_PWM[2] = pwm_flat;
    //     Sleep(100);
    //     std::cout << "pwm_flat:  " << pwm_flat<<" dir:    " <<dir<<std::endl;
  
    // }
}
