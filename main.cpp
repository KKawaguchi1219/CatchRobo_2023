#include "mbed.h"
#include "math.h"
#include "Encoder_InterruptIn.h"
#include "WiiClassicController.h"
#include "Servo6221MG.h"


// wiiリモコン
WiiClassicController wii(D14, D15); // SDA(left side on rev C, GREEN), SCL(D6, YELLOW)

// コントローラー値
#define I2C_UPDATE_TIME 1.0
#define JYOY_L_CENTER 31
#define JYOY_R_CENTER 16
#define JYOY_MARGIN 4
#define JYOY_MARGIN2 3
#define x_gain_1 0.05;
#define y_gain_1 0.05;
#define x_gain_2 0.02;
#define y_gain_2 0.02;

bool b_A;
bool b_B;
bool b_X;
bool b_Y;
bool b_minus;
bool b_plus;
bool b_home;
bool b_DU;
bool b_DD;

unsigned char j_LX;
unsigned char j_LY;
unsigned char j_RX;
unsigned char j_RY;

// アーム回転用モータ: TIM1
PwmOut pwm1(PA_8);
PwmOut pwm2(PA_10);
float vol=0.0f;

// PID
float target_count = 0.0f;
float target_transition = 0.0f;
#define command 25250.0f
float target_plus = command;
float target_minus = -command;
int target_flag = 0;

float diff[2];
#define Kp 0.0245f           // pゲイン 
#define Ki 0.09f             // iゲイン     
#define Kd 0.0003f           // dゲイン   
#define Krc 0.30f            // 指数平均ゲイン
#define limit_duty 0.44f
static float pre_i = 0.0f;
float p, i, d;

// encoder
Encoder encoder(PA_0, PA_1);
float encoder_count = 0.0f;

//x軸モータ: TIM2
PwmOut pwm_x1(PB_10);
PwmOut pwm_x2(PB_3);
float vol1=0.0f;
//y軸モータ: TIM3
PwmOut pwm_y1(PB_4);
PwmOut pwm_y2(PB_5);
float vol2=0.0f;

// 空気圧バルブ(ハンド)
DigitalOut valve1(PC_3);
DigitalOut valve2(PC_2);
DigitalOut valve3(PB_7);
DigitalOut hand[3] = {valve1, valve2, valve3};

// サーボ
Servo6221MG servo(PC_8);

// リミットスイッチ
DigitalIn limit1(PC_12);

// xy移動用関数
void x_move(unsigned char& j_LX);
void y_move(unsigned char& j_RY);

// ハンド制御
void hand_control(bool& b, int i);

// Timer割込み
Ticker flipper;
#define time 1  // time [ms]

Timer t;
int tim=0;
static BufferedSerial serial_port(USBTX, USBRX);

// PID制御
float pid_motor(float count, float target){
    #define delta 0.001f
    #define delta_inv 1000.0f // d項のdeltaを乗算用に調整

    static float integral = 0.0f;
    float d_tmp;              // 差分格納用
    static float d_factor = 0.0f;

    float out;

    diff[0] = diff[1];
    diff[1] = target - count;

    p = Kp * diff[1];

    pre_i = integral;
    integral += (diff[1] + diff[0]) * 0.50 * delta;
    i = Ki * integral;

    d_tmp = (diff[1] - diff[0])*delta_inv;
    d_factor = Krc*d_tmp + (1-Krc)*d_factor;

    d = Kd*d_factor;
    out = p + i + d;

    if(out > limit_duty){
        out = limit_duty;
        integral = pre_i;
    }else if(out < -limit_duty){
        out = -limit_duty;
        integral = pre_i;
    }

    return out;
}

// 回転制御 割込み関数
void motor_control(){
    if(target_flag == 1 && target_transition < target_plus){
        if(target_transition < 0) target_transition=0;
        tim = t.read_ms();

        if(tim<=3000){
            target_transition += 7.21;
        }else if(tim>3000 && tim <= 4000){
            target_transition = -0.00362*pow(tim-4000.0, 2.0) + target_plus;
        }else{
            target_transition = target_plus;
        }

    }else if (target_flag == -1 && target_transition > target_minus){
        if(target_transition > 0) target_transition=0;
        tim = t.read_ms();

        if(tim<=3000){
            target_transition -= 7.21;
        }else if(tim>3000 && tim <= 4000){
            target_transition = 0.00362*pow(tim-4000.0, 2.0) + target_minus;
        }else{
            target_transition = target_minus;
        }

    }
    encoder_count = encoder.get_encoder_count();
    vol = (pid_motor(encoder_count, target_transition));
    pwm1.write(0.50f -vol);
    pwm2.write(0.50f +vol);
}

int main(void){ 
    int flag=1;
    printf("Program_Start! \n");
    
    flipper.attach(&motor_control, chrono::milliseconds(time));

    pwm1.period_us(100);
    pwm2.period_us(100);
    pwm1.write(0.50f);
    pwm2.write(0.50f);
    
    pwm_x1.period_us(100);
    pwm_x2.period_us(100);
    // pwmはcenter alignするので両方0.50でブレーキ状態
    pwm_x1.write(0.50f);
    pwm_x2.write(0.50f);

    pwm_y1.period_us(100);
    pwm_y2.period_us(100);
    // pwmはcenter alignするので両方0.50でブレーキ状態
    pwm_y1.write(0.50f);
    pwm_y2.write(0.50f);

    // center align
    TIM1->CR1 |= TIM_CR1_CMS_0;
    TIM2->CR1 |= TIM_CR1_CMS_0;
    TIM3->CR1 |= TIM_CR1_CMS_0;
    TIM4->CR1 |= TIM_CR1_CMS_0;

    serial_port.set_baud(115200);

    while(1){
        // コントローラー入力
        b_A = wii.button_A();
        b_B = wii.button_B();
        b_X = wii.button_X();
        b_Y = wii.button_Y();

        b_plus = wii.button_plus();
        b_minus = wii.button_minus();
        b_home = wii.button_home();

        b_DD = wii.button_DD();
        b_DU = wii.button_DU();

        j_LX = wii.joy_LX();
        j_LY = wii.joy_LY();
        j_RX = wii.joy_RX();
        j_RY = wii.joy_RY();

        // 順回転の命令
        if(b_plus){
            //printf("b_plus pressed\r\n");
            target_flag = 1;
            t.start();
        }
        // 逆回転の命令
        if(b_minus){
            //printf("b_minus pressed\r\n");
            target_flag = -1;
            t.start();
        }

        // エンコーダの値をリセット
        if(b_home){
            printf("b_home pressed\r\n");
            t.stop();
            t.reset();
            target_flag = 0;
            target_transition=0;
            encoder.reset_count();
        }

        // ハンド制御部
        hand_control(b_A, 0);
        hand_control(b_X, 1);
        hand_control(b_Y, 2);

        // ジョイスティックの入力を非線形に -0.5 - +0.5 の間に変換(x方向の制御)
        x_move(j_LX);
        // ジョイスティックの入力を非線形に -0.5 - +0.5 の間に変換(y方向の制御)
        y_move(j_RY);
/*      
        if(limit1){
            pwm_x1.write(0.50f);
            pwm_x2.write(0.50f);
            HAL_Delay(100);
            
            //pwm_x1.write(0.70f);
            //pwm_x2.write(0.30f);
            //HAL_Delay(100);
            //pwm_x1.write(0.50f);
            //pwm_x2.write(0.50f);
            
        }*/        
        /*
        if(b_B){
            flag=0;
            printf("timer_stop\r\n");
            t.stop();
            printf("%d\r\n", encoder.get_encoder_count());
        }
        */
        
        // debug
        if(flag==1){
            printf("%d,%f,%d\r\n",t.read_ms(), target_transition, encoder.get_encoder_count());
            //printf("pre_integral: %f\r\n", pre_i);
            //printf("p: %f, i: %f, d: %f\r\n", p, i, d);
            //HAL_Delay(200);
        }


        //HAL_Delay(200);
    }
/*
        printf("j_LX: %d\r\n", j_LX - JYOY_L_CENTER-2);
        printf("j_RY: %d\r\n", j_RY - JYOY_R_CENTER-1);
        printf("Volum_x: %f\r\n", vol1);
        printf("Volum_y: %f\r\n", vol2);
        printf("limit1: %d\r\n", limit1.read());
       HAL_Delay(100);
*/


    return 0;
}

void x_move(unsigned char& j_LX){
    if((j_LX - JYOY_L_CENTER-2) != 0){
            // 変換式
            vol1 = 1.0/(1.0+exp(-0.09*(j_LX-JYOY_L_CENTER-2))) - 1.0/2.0;
            //float vol = (1.0/tanh(j_LX - JYOY_L_CENTER - 2) - 1.0/(j_LX - JYOY_L_CENTER - 2)) / 2.0;
            if(vol1 >= 0.48){
                vol1 = 0.48;
            }else if(vol1 <= -0.48){
                vol1 = -0.48;
            }
            pwm_x1.write(0.50 -vol1);
            pwm_x2.write(0.50 +vol1);
            HAL_Delay(100);

        }else{
            pwm_x1.write(0.50f);
            pwm_x2.write(0.50f);
            HAL_Delay(100);

        }
}

void y_move(unsigned char& j_RY){
    if((j_RY - JYOY_R_CENTER-1) != 0){
            // 変換式
            vol2 = (1.0/(1.0+exp(-0.09*(j_RY-JYOY_R_CENTER-1))) - 1.0/2.0)*1.7;
            //float vol = (1.0/tanh(j_LX - JYOY_L_CENTER - 2) - 1.0/(j_LX - JYOY_L_CENTER - 2)) / 2.0;
            if(vol2 >= 0.48){
                vol2 = 0.48;
            }else if(vol2 <= -0.48){
                vol2 = -0.48;
            }
            pwm_y1.write(0.50 -vol2);
            pwm_y2.write(0.50 +vol2);
            HAL_Delay(100);

        }else{
            pwm_y1.write(0.50f);
            pwm_y2.write(0.50f);
            HAL_Delay(100);

        }
}

void hand_control(bool& b, int i){
    if(b){
        printf("b_A pressed!\r\n");
        hand[i] = !hand[i];
        printf("valve%d: %d\r\n",i, hand[i].read());
        // ここの値を変えて入力の受付時間を調整
        HAL_Delay(300);
    }
}
