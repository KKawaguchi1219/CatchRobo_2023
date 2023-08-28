#ifndef encoder_H
#define encoder_H

#include "mbed.h"

class Encoder
{
    private:
        // エンコーダ系の変数
        DigitalIn _pin1;
        DigitalIn _pin2;
        InterruptIn _pin1_i;
        InterruptIn _pin2_i;
        
        int encoder_count;

        // 割込み関数
        void counterArise();
        void counterAfall();
        void counterBrise();
        void counterBfall();

    public:
        Encoder(PinName pin1, PinName pin2);
        /* 初期化処理 */
        void init();
        /*エンコーダの値を返す*/
        int get_encoder_count();

        /*エンコーダの値をリセット*/
        void reset_count();

};

#endif