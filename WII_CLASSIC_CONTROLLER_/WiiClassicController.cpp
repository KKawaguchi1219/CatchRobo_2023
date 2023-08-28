#include "mbed.h"
#include "WiiClassicController.h"
WiiClassicController::WiiClassicController(PinName p_sda, PinName p_scl):i2c(p_sda, p_scl)
{
    WiiClaInitFlag = init();
    timer.start();
}
bool WiiClassicController::init()
{
    bool result = false;
    unsigned char cmd[] = {CLASSIC_REGADDR, 0x00};
    if (i2c.write(CLASSIC_ADDR, (const char*)cmd, sizeof(cmd)) == I2C_ACK) {
        result = true;
    }
    return result;
}
void WiiClassicController::read()
{
    int i;
    if(timer.read_ms() < 50) return;
    if(!WiiClaInitFlag) {
        WiiClaInitFlag = init();
    }
    if(WiiClaInitFlag) {
        const unsigned char cmd[] = {0x00};
        if (i2c.write(CLASSIC_ADDR, (const char*)cmd, sizeof(cmd)) == I2C_ACK) {
            //wait(I2C_READ_DELAY);
            ThisThread::sleep_for(10ms);
            if (i2c.read(CLASSIC_ADDR, readBuf, sizeof(readBuf)) == I2C_ACK) {
                for(i = 0; i < CLASSIC_READLEN; ++i) {
                    readBuf[i] = (readBuf[i] ^ 0x17) + 0x17;
                }
                    if(readBuf[0] & 0x80) joyRX = 0x10;
                    else    joyRX = 0;
                    if(readBuf[0] & 0x40) joyRX += 0x08;
                    if(readBuf[1] & 0x80) joyRX += 0x04;
                    if(readBuf[1] & 0x40) joyRX += 0x02;
                    if(readBuf[1] & 0x80) joyRX += 0x01;
                    joyRY = readBuf[2] & 0x1F;
                    joyLX = readBuf[0] & 0x3F;
                    joyLY = readBuf[1] & 0x3F;
                if(readBuf[4] & 0x80) {
                    buttonDR = 0;
                } else {
                    buttonDR = 1;
                }
                if(readBuf[4] & 0x40) {
                    buttonDD = 0;
                } else {
                    buttonDD = 1;
                }
                if(readBuf[4] & 0x20) {
                    buttonL = 0;
                } else {
                    buttonL = 1;
                }
                if(readBuf[4] & 0x10) {
                    buttonMinus = 0;
                } else {
                    buttonMinus = 1;
                }
                if(readBuf[4] & 0x08) {
                    buttonHome = 0;
                } else {
                    buttonHome = 1;
                }
                if(readBuf[4] & 0x04) {
                    buttonPlus = 0;
                } else {
                    buttonPlus = 1;
                }
                if(readBuf[4] & 0x02) {
                    buttonR = 0;
                } else {
                    buttonR = 1;
                }
                if(readBuf[5] & 0x80) {
                    buttonZL = 0;
                } else {
                    buttonZL = 1;
                }
                if(readBuf[5] & 0x40) {
                    buttonB = 0;
                } else {
                    buttonB = 1;
                }
                if(readBuf[5] & 0x20) {
                    buttonY = 0;
                } else {
                    buttonY = 1;
                }
                if(readBuf[5] & 0x10) {
                    buttonA = 0;
                } else {
                    buttonA = 1;
                }
                if(readBuf[5] & 0x08) {
                    buttonX = 0;
                } else {
                    buttonX = 1;
                }
                if(readBuf[5] & 0x04) {
                    buttonZR = 0;
                } else {
                    buttonZR = 1;
                }
                if(readBuf[5] & 0x02) {
                    buttonDL = 0;
                } else {
                    buttonDL = 1;
                }
                if(readBuf[5] & 0x01) {
                    buttonDU = 0;
                } else {
                    buttonDU = 1;
                }
            }
        }
    }
    timer.reset();
}
unsigned char WiiClassicController::joy_RX()
{
    read();
    return joyRX;
}
unsigned char WiiClassicController::joy_RY()
{
    read();
    return joyRY;
}
unsigned char WiiClassicController::joy_LX()
{
    read();
    return joyLX;
}
unsigned char WiiClassicController::joy_LY()
{
    read();
    return joyLY;
}
bool WiiClassicController::button_A()
{
    read();
    return buttonA;
}
bool WiiClassicController::button_B()
{
    read();
    return buttonB;
}
bool WiiClassicController::button_X()
{
    read();
    return buttonX;
}
bool WiiClassicController::button_Y()
{
    read();
    return buttonY;
}
bool WiiClassicController::button_minus()
{
    read();
    return buttonMinus;
}
bool WiiClassicController::button_plus()
{
    read();
    return buttonPlus;
}
bool WiiClassicController::button_home()
{
    read();
    return buttonHome;
}
bool WiiClassicController::button_R()
{
    read();
    return buttonR;
}
bool WiiClassicController::button_L()
{
    read();
    return buttonL;
}
bool WiiClassicController::button_ZL()
{
    read();
    return buttonZL;
}
bool WiiClassicController::button_ZR()
{
    read();
    return buttonZR;
}
bool WiiClassicController::button_DU()
{
    read();
    return buttonDU;
}
bool WiiClassicController::button_DD()
{
    read();
    return buttonDD;
}
bool WiiClassicController::button_DL()
{
    read();
    return buttonDL;
}
bool WiiClassicController::button_DR()
{
    read();
    return buttonDR;
}
