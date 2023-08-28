/***************************************** 
* Servo6221MG.h
 *****************************************/
#ifndef SV_6221MG_H
#define SV_6221MG_H
#include "mbed.h"
#include "PwmOut.h"

class Servo6221MG: public PwmOut
{
public:
    /* コンストラクタ */
    Servo6221MG( PinName  port ):PwmOut( port ){}
    /* 初期化処理 */
    void init();
    /* 回転処理 */
    int  roll( unsigned int angle );
};

#endif 