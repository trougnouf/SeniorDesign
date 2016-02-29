#include "MLX90393.h"
#include "mbed.h"


MLX90393::MLX90393(PinName SlaveSelect, SPI* spi) : _SlaveSelect(SlaveSelect)
{
    this->spi = spi;
    _SlaveSelect = 1;
}

MLX90393::MLX90393(int I2CAddress, I2C* i2c) : _SlaveSelect(NC)
{
    this->i2c = i2c;
    _I2CAddress = I2CAddress;
}

//*************************************** MAIN FUNCTIONS ***************************************

void MLX90393::EX(char *receiveBuffer, int mode)
{
    write_buffer[0] = 0x80;
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 1, 1);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 1, 1);
    }
}

void MLX90393::SB(char *receiveBuffer, char zyxt, int mode)
{
    write_buffer[0] = (0x10)|(zyxt);
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 1, 1);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 1, 1);
    }
}

void MLX90393::SWOC(char *receiveBuffer, char zyxt, int mode)
{
    write_buffer[0] = (0x20)|(zyxt);
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 1, 1);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 1, 1);
    }
}

void MLX90393::SM(char *receiveBuffer, char zyxt, int mode)
{
    write_buffer[0] = (0x30)|(zyxt);
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 1, 1);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 1, 1);
    }
}

void MLX90393::RM(char *receiveBuffer, char zyxt, int mode)
{
    write_buffer[0] = (0x40)|(zyxt);
    for(int i=0; i<2*count_set_bits(zyxt); i++) {
        write_buffer[i+2] = 0x00;
    }
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 1, 1+2*count_set_bits(zyxt));
    } else {
        Send_I2C(receiveBuffer, write_buffer, 1, 1+2*count_set_bits(zyxt));
    }
}

void MLX90393::RR(char *receiveBuffer, int address, int mode)
{
    write_buffer[0] = 0x50;
    write_buffer[1] = address << 2;
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 2, 3);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 2, 3);
    }
}

void MLX90393::WR(char *receiveBuffer, int address, int data, int mode)
{
    write_buffer[0] = 0x60;
    write_buffer[1] = (data&0xFF00)>>8;
    write_buffer[2] = data&0x00FF;
    write_buffer[3] = address << 2;
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 4, 1);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 4, 1);
    }
}

void MLX90393::RT(char *receiveBuffer, int mode)
{
    write_buffer[0] = 0xF0;
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 1, 1);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 1, 1);
    }
}

void MLX90393::NOP(char *receiveBuffer, int mode)
{
    write_buffer[0] = 0x00;
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 1, 1);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 1, 1);
    }
}

void MLX90393::HR(char *receiveBuffer, int mode)
{
    write_buffer[0] = 0xD0;
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 1, 1);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 1, 1);
    }
}

void MLX90393::HS(char *receiveBuffer, int mode)
{
    write_buffer[0] = 0xE0;
    if (mode == 1) {
        Send_SPI(receiveBuffer, write_buffer, 1, 1);
    } else {
        Send_I2C(receiveBuffer, write_buffer, 1, 1);
    }
}

//************************************* COMMUNICATION LEVEL ************************************

void MLX90393::Send_SPI(char *receiveBuffer, char *sendBuffer, int sendMessageLength, int receiveMessageLength)
{
    char* tempSendBuffer = sendBuffer;
    char* tempReceiveBuffer = receiveBuffer;
    _SlaveSelect = 0;
    for(int i=0; i<sendMessageLength; i++) {
        spi->write(*tempSendBuffer);
        tempSendBuffer++;
    }
    for(int i=0; i<receiveMessageLength; i++) {
        *receiveBuffer = spi->write(0x00);
        receiveBuffer++;
    }
    _SlaveSelect = 1;
    receiveBuffer = tempReceiveBuffer;
}

void MLX90393::Send_I2C(char *receiveBuffer, char *sendBuffer, int sendMessageLength, int receiveMessageLength)
{
    char* tempSendBuffer = sendBuffer;
    char* tempReceiveBuffer = receiveBuffer;
    i2c->write(_I2CAddress, tempSendBuffer, sendMessageLength, true);
    i2c->read(_I2CAddress, receiveBuffer, receiveMessageLength);
    receiveBuffer = tempReceiveBuffer;
}

//*************************************** EXTRA FUNCTIONS **************************************

int MLX90393::count_set_bits(int N)
{
    int result = 0;
    while(N) {
        result++;
        N &=N-1;
    }
    return result;
}



