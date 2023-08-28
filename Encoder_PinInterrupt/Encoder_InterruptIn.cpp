#include "Encoder_InterruptIn.h"
#include "mbed.h"

Encoder::Encoder(PinName pin1, PinName pin2):
    _pin1(pin1), 
    _pin2(pin2), 
    _pin1_i(pin1), 
    _pin2_i(pin2)
{
    encoder_count = 0;
    _pin1_i.rise(callback(this, &Encoder::counterArise));
    _pin1_i.fall(callback(this, &Encoder::counterAfall));
    _pin2_i.rise(callback(this, &Encoder::counterBrise));
    _pin2_i.fall(callback(this, &Encoder::counterBfall));
}


int Encoder::get_encoder_count(){
    return encoder_count;
}

void Encoder::reset_count(){
    encoder_count = 0;
}

void Encoder::counterArise(){
    if (_pin2 == 1) {
        encoder_count--;
    }else{
        encoder_count++;
    }
}

void Encoder::counterAfall(){
    if (_pin2 == 1) {
        encoder_count++;
    }else{
        encoder_count--;
    }
}

void Encoder::counterBrise(){
    if (_pin1 == 1) {
        encoder_count++;
    }else{
        encoder_count--;
    }
}

void Encoder::counterBfall(){
    if (_pin1 == 1) {
        encoder_count--;
    }else{
        encoder_count++;
    }
}
