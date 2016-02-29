#ifndef MBED_MLX90393_H
#define MBED_MLX90393_H

#include "mbed.h"
/** MLX90393 class.
 *  Used for communication with the MLX90393 in both I2C and SPI.
 *  AKN/JVV rev.1 - JUL2014
 */
class MLX90393
{

public:


    /**[Constructor] Create MLX90393 instance, using SPI for communication.
    * @param SlaveSelect SlaveSelect pin.
    * @param spi Reference to SPI.
     */
    MLX90393(PinName SlaveSelect, SPI* spi);


    /**[Constructor] Create MLX90393 instance, using I2C for communication.
    * @param Address I2C Address of the MLX90393.
    * @param spi Reference to I2C.
    */
    MLX90393(int Address, I2C* i2c);


    /** Send 'exit' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte only.
    */
    void EX(char *receiveBuffer, int mode);


    /** Send 'start burst mode' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte only.
    */
    void SB(char *receiveBuffer, char zyxt, int mode);


    /** Send 'start wake-up on change mode' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param zyxt Selection of the axes/temperature to which the mode should apply.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte only.
    */
    void SWOC(char *receiveBuffer, char zyxt, int mode);


    /** Send 'single measurement' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param zyxt Selection of the axes/temperature to be measured.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte only.
    */
    void SM(char *receiveBuffer, char zyxt, int mode);


    /** Send 'read measurement' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param zyxt Selection of the axes/temperature to be read out.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte, followed by 2 bytes for each T, X, Y and Z (depending on zyxt, some can be left out).
    */
    void RM(char *receiveBuffer, char zyxt, int mode);


    /** Send 'read register' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param address The register to be read out.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte, followed by 2 bytes for the data at the specific register.
    */
    void RR(char *receiveBuffer, int address, int mode);


    /** Send 'write register' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param address The register to be written.
    * @param data The 16-bit word to be written in the register.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will only contain the status byte.
    */
    void WR(char *receiveBuffer, int address, int data, int mode);


    /** Send 'reset' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte only.
    */
    void RT(char *receiveBuffer, int mode);


    /** Send 'NOP' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte only.
    */
    void NOP(char *receiveBuffer, int mode);


    /** Send 'memory recall' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte only.
    */
    void HR(char *receiveBuffer, int mode);


    /** Send 'memory store' command to MLX90393.
    * @param *receiveBuffer Pointer to receiveBuffer, will contain response of IC after command is sent.
    * @param mode Communication mode (0=I2C, 1=SPI).
    * @note The receiveBuffer will contain the status byte only.
    */
    void HS(char *receiveBuffer, int mode);
    int count_set_bits(int N);

    //Pulic statics, class variables
    static const int spi_mode =3;
    static const int spi_bits =8;
    static const char i2c_address = 0x19;


private:
//SPI
    SPI* spi;
    void Send_SPI(char *receiveBuffer, char *sendBuffer, int sendMessageLength, int receiveMessageLength);
//I2C
    I2C* i2c;
    void Send_I2C(char *receiveBuffer, char *sendBuffer, int sendMessageLength, int receiveMessageLength);
    int _I2CAddress;
//Shared
    DigitalOut _SlaveSelect;
    char write_buffer[10];


};

#endif
