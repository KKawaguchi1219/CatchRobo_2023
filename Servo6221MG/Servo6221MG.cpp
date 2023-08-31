#include "Servo6221MG.h"

#define DS_PERIOD 20     /* 20[msec] */
#define DS_NEUTRAL 500  /* 0° 500[usec] */

/* 初期化処理 */
void Servo6221MG::init(){
    /* 周期の設定を行う */
    period_ms( DS_PERIOD );
    /* 0°に角度調整 */
    pulsewidth_us( DS_NEUTRAL );
    pulsewidth_us( 1500 );

}

/* 回転処理 */
int Servo6221MG::roll( unsigned int angle ){
    double lPulseWidth ;
    if (angle <= 180)
        lPulseWidth = (angle / 180.0)* 2000 + 500;     //lPulseWidth = (angle / 180.0) * 2000 + 500;
    else
        return -1;
    
    pulsewidth_us( (int)lPulseWidth );
    return 0;
    
}