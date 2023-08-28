/*********************************
TOTTORI_U
ROBOT_LABO
2016/6/16
**********************************/
#ifndef WII_CLASSIC_CONTROLLER_H
#define WII_CLASSIC_CONTROLLER_H
#include "mbed.h"
// I2C
#define CLASSIC_ADDR     0xA4  // 0x52 << 1
#define CLASSIC_REGADDR  0x40  //
#define CLASSIC_READLEN  0x06  //
#define I2C_ACK 0
#define I2C_READ_DELAY  0.01
class WiiClassicController{
public:
    WiiClassicController(PinName p_sda, PinName p_scl);
    //read();
    unsigned char joy_LX();
    unsigned char joy_LY();
    unsigned char joy_RX();
    unsigned char joy_RY();
    bool button_DL();
    bool button_DR();
    bool button_DD();
    bool button_DU();
    bool button_minus();
    bool button_plus();
    bool button_home();
    bool button_B();
    bool button_Y();
    bool button_A();
    bool button_X();
    bool button_R();
    bool button_L();
    bool button_ZL();
    bool button_ZR();
private:
    I2C i2c;
    Timer timer;
    bool WiiClaInitFlag; //= false;
    char readBuf[CLASSIC_READLEN];
    unsigned char joyRX, joyRY, joyLX, joyLY;
    bool buttonA, buttonB, buttonX,
            buttonY, buttonR, buttonL,
            buttonZR, buttonZL, buttonDU,
            buttonDD, buttonDL, buttonDR,
            buttonPlus, buttonMinus, buttonHome;
    bool init();
    void read();
};
#endif