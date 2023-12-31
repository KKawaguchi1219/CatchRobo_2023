#include "mbed.h"
#include "math.h"
#include <stdint.h>
#include "Encoder_InterruptIn.h"
#include "WiiClassicController.h"
#include "Servo6221MG.h"
#include <cstdint>

// wiiリモコン
WiiClassicController wii(D14, D15); // SDA(PB_8, GREEN), SCL(PB_9, BLUE)

// コントローラー値
#define I2C_UPDATE_TIME 1.0
#define JYOY_L_CENTER 33
#define JYOY_R_CENTER 17
#define xy_margin 1.1f             // 現時点(9/7)でvol=±0.45fが最高なので, 変換式の最後に1.1倍して±0.495fぐらいを最高にしてみては？

bool b_A;
bool b_B;
bool b_X;
bool b_Y;
bool b_R;
bool b_L;
bool b_ZL;
bool b_ZR; 
bool b_minus;
bool b_plus;
bool b_home;
bool b_DU;
bool b_DD;

unsigned char j_LX;
unsigned char j_LY;
unsigned char j_RX;
unsigned char j_RY;

DigitalOut led(LED1);

// アーム回転用モータ: TIM1
PwmOut pwm_rot1(PA_8);
PwmOut pwm_rot2(PA_10);
float vol_rot=0.0f;

// PID
float target_count = 0.0f;
float target_transition = 0.0f;

#define command 25400.0f
float target_plus = command;
float target_minus = -command;
float t_base = command*0.1386962552, t1 = t_base-500, t2 = t_base+500;
//a = (7.21*t1-command)/(t1-t2)*(t1-t2);
float a = fabs((7.21*t1-command)*0.000001);
int target_flag = 0;

#define Kp 0.0245f           // pゲイン 
#define Ki 0.09f             // iゲイン     
#define Kd 0.0003f           // dゲイン   
#define Krc 0.30f            // 指数平均ゲイン
#define limit_duty 0.48f
float diff[2];
static float pre_i = 0.0f;

// encoder
Encoder encoder(PA_0, PA_1);
float encoder_count = 0.0f;

//x軸モータ: TIM2
PwmOut pwm_x1(PB_10);
PwmOut pwm_x2(PB_3);
float vol_x=0.0f;
//y軸モータ: TIM3
PwmOut pwm_y1(PB_4);
PwmOut pwm_y2(PB_5);
float vol_y=0.0f;

// 空気圧バルブ(ハンド)
DigitalOut valve1(PC_3);
DigitalOut valve2(PC_2);
DigitalOut valve3(PB_7);

// サーボ
Servo6221MG servo1(PB_6);
int each_range = 90;
int p1=90;
int temp_angle_1=90;

// リミットスイッチ
DigitalIn limit1_yminus(PC_12);
DigitalIn limit2_yplus(PC_10);
DigitalIn limit3_xminus(PC_11);
DigitalIn limit4_xplus(PD_2);
DigitalIn limit5(PC_7);

// raspi通信用
DigitalOut signal1(PC_4);
DigitalOut signal2(PA_12);

// xy移動用関数
void x_move(unsigned char& j_LX, DigitalIn& limit_yminus, DigitalIn& limit_plus);
void y_move(unsigned char& j_RY, DigitalIn& limit_yminus, DigitalIn& limit_plus);

// ServoMotor制御
void servo_rot_p(bool& button, int& p, int& temp_angle, Servo6221MG& servo);
void servo_rot_n(bool& button, int& p, int& temp_angle, Servo6221MG& servo);

// ハンド制御
void hand_control(bool& b, DigitalOut& hand, int i);

// Timer割込み
Ticker flipper;
#define time 1  // time [ms]

Timer t;
uint32_t tim=0;

//　デバッグ出力用
static BufferedSerial serial_port(USBTX, USBRX);

// PID制御
float pid_motor(float& count, float& target){
    #define delta 0.001f
    #define delta_inv 1000.0f // d項のdeltaを乗算用に調整
    float p, i, d;

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
    t1 = (int) t1, t2=(int)t2;
    if(target_flag == 1 && target_transition < target_plus){
        if(target_transition < 0) target_transition=0;
        tim = t.read_ms();

        if(tim<=t1){
            target_transition += 7.21;
        }else if(tim>t1 && tim <= t2){
            target_transition = -a*pow(tim-t2, 2.0) + target_plus;
        }else{
            target_transition = target_plus;
        }

    }else if (target_flag == -1 && target_transition > target_minus){
        if(target_transition > 0) target_transition=0;
        tim = t.read_ms();

        if(tim<=t1){
            target_transition -= 7.21;
        }else if(tim>t1 && tim <= t2){
            target_transition = a*pow(tim-t2, 2.0) + target_minus;
        }else{
            target_transition = target_minus;
        }

    }
    encoder_count = encoder.get_encoder_count();
    vol_rot = (pid_motor(encoder_count, target_transition));
    pwm_rot1.write(0.50f -vol_rot);
    pwm_rot2.write(0.50f +vol_rot);
}

int main(void){ 

    printf("Program_Start! \r\n");
    int flag=1;
    serial_port.set_baud(115200);

    // pid制御用割込み関数
    flipper.attach(&motor_control, chrono::milliseconds(time));

    limit1_yminus.mode(PullDown);
    limit2_yplus.mode(PullDown);
    limit3_xminus.mode(PullDown);
    limit4_xplus.mode(PullDown);

    // ハンド初期化
    valve1=0;
    valve2=0;
    valve3=0;

    // サーボ初期化
    servo1.init();

    // アーム回転用DCモータ
    pwm_rot1.period_us(100);
    pwm_rot2.period_us(100);
    pwm_rot1.write(0.50f);
    pwm_rot2.write(0.50f);
    
    // ハンド移動用(xy軸移動用)DCモータ
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

    // PWMのcenter align
    TIM1->CR1 |= TIM_CR1_CMS_0;
    TIM2->CR1 |= TIM_CR1_CMS_0;
    TIM3->CR1 |= TIM_CR1_CMS_0;
    TIM4->CR1 |= TIM_CR1_CMS_0;

    while(1){
        // コントローラー入力
        b_A = wii.button_A();
        b_B = wii.button_B();
        b_X = wii.button_X();
        b_Y = wii.button_Y();

        b_R = wii.button_R();
        b_L = wii.button_L();
        b_ZR = wii.button_ZR();
        b_ZL = wii.button_ZL();
        
        b_plus = wii.button_plus();
        b_minus = wii.button_minus();
        b_home = wii.button_home();

        b_DD = wii.button_DD();
        b_DU = wii.button_DU();

        j_LX = wii.joy_LX();
        j_LY = wii.joy_LY();
        j_RX = wii.joy_RX();
        j_RY = wii.joy_RY();

        // アーム正回転の命令
        if(b_plus){
            //printf("b_plus pressed\r\n");
            target_flag = 1;
            t.start();
        }
        // アーム逆回転の命令
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

        // 角度微調整用
        if(b_DU){
            target_transition++;
        }else if(b_DD){
            target_transition--;
        }

        // raspi通信
        if(b_ZR){
            signal1 = !signal1;
            HAL_Delay(500);
        }else if (b_ZL) {
            signal2 = !signal2;
            HAL_Delay(500);
        }
        //printf("sig1: %d, sig2: %d\r\n", signal1.read(), signal2.read());

        // ハンド制御部
        hand_control(b_A, valve1, 1);
        hand_control(b_X, valve2, 2);
        hand_control(b_Y, valve3, 3);

        // b_Lを押すごとに90°回転．ただし, 180°未満の場合のみ
        servo_rot_p(b_L, p1, temp_angle_1, servo1);
        // b_Rを押すごとに-90°回転．ただし, 180°以上の場合のみ
        servo_rot_n(b_R, p1, temp_angle_1, servo1);

        // ジョイスティックの入力を非線形に -0.5 - +0.5 の間に変換(x方向の制御)
        x_move(j_LX, limit3_xminus, limit4_xplus);
        // ジョイスティックの入力を非線形に -0.5 - +0.5 の間に変換(y方向の制御)
        y_move(j_RY, limit1_yminus, limit2_yplus);
    
        
        if(b_B){
            flag=0;
            printf("timer_stop\r\n");
            t.stop();
            printf("%d\r\n", encoder.get_encoder_count());
        }
        
        
        // debug
        
        if(flag==1){
            printf("%d,%f,%d\r\n",t.read_ms(), target_transition, encoder.get_encoder_count());
            //HAL_Delay(200);
        }
        /*
        printf("j_LX: %d\r\n", j_LX - JYOY_L_CENTER);
        printf("j_RY: %d\r\n", j_RY - JYOY_R_CENTER);
        printf("Volum_x: %f\r\n", vol_x);
        printf("Volum_y: %f\r\n", vol_y);
        
        HAL_Delay(10);
        */
    }


    return 0;
}

void x_move(unsigned char& j_LX, DigitalIn& limit_xminus, DigitalIn& limit_xplus){
    float x = j_LX - JYOY_L_CENTER;
    float x1 = 8.0f;
    float x2 = 18.0f;
    float x3 = 26.0f;

    if(limit_xminus){
        printf("limit3: push\r\n");
        pwm_x1.write(0.40f);
        pwm_x2.write(0.50f);
        HAL_Delay(500);
        pwm_x1.write(0.50f);
        pwm_x2.write(0.50f);
        HAL_Delay(500);
    }else if(limit_xplus){
        printf("limit4: push\r\n");
        pwm_x1.write(0.50f);
        pwm_x2.write(0.40f);
        HAL_Delay(500);
        pwm_x1.write(0.50f);
        pwm_x2.write(0.50f);
        HAL_Delay(500);
    }

    if(x != 0.0f){
        // 変換式...もっとコンパクトにまとめたほうが良いと思います
        if(x>0.0f && x<=x1){
            vol_x = 0.0015625*x*x;
        }else if(x>x1 && x<=x2){
            vol_x = 0.025*(x-20.0) + 0.35; 
        }else if (x>x2 && x<=x3) {
            vol_x = -0.0015625*(x-26.0)*(x-26.0) + 0.45;
        }else if(x<0.0f && x>=-x1){
            vol_x = -0.0015625*x*x;
        }else if(x<-x1 && x>=-x2){
            vol_x = 0.025*(x+20.0) - 0.35; 
        }else if (x<-x2 && x>=-x3) {
            vol_x = 0.0015625*(x+26.0)*(x+26.0) - 0.45;
        }
        // 出力のさらに調整...上の変換式を練り直すのがめんどくｓ
        vol_x *= xy_margin;

        if(vol_x >= 0.45*xy_margin){
            vol_x = 0.45*xy_margin;
        }else if(vol_x <= -0.45*xy_margin){
            vol_x = -0.45*xy_margin;
        }
        pwm_x1.write(0.50 +vol_x);
        pwm_x2.write(0.50 -vol_x);
    }else{
        pwm_x1.write(0.50f);
        pwm_x2.write(0.50f);
    }
}

void y_move(unsigned char& j_RY, DigitalIn& limit_yminus, DigitalIn& limit_yplus){
    float y = 2.0*(j_RY - JYOY_R_CENTER);
    float y1 = 8.0f;
    float y2 = 18.0f;
    float y3 = 26.0f;

    if(limit1_yminus){
        printf("limit1: push\r\n");
        pwm_y1.write(0.40f);
        pwm_y2.write(0.50f);
        HAL_Delay(500);
        pwm_y1.write(0.50f);
        pwm_y2.write(0.50f);
        HAL_Delay(500);
    }else if(limit2_yplus){
        printf("limit2: push\r\n");
        pwm_y1.write(0.50f);
        pwm_y2.write(0.40f);
        HAL_Delay(500);
        pwm_y1.write(0.50f);
        pwm_y2.write(0.50f);
        HAL_Delay(500);
    }

    if(y != 0){
        if(y>0) y+=1;

        // 変換式
        if(y>0.0f && y<=y1){
            vol_y = 0.0015625*y*y;
        }else if(y>y1 && y<=y2){
            vol_y = 0.025*(y-20.0) + 0.35; 
        }else if (y>y2 && y<=y3) {
            vol_y = -0.0015625*(y-26.0)*(y-26.0) + 0.45;
        }else if(y<0.0f && y>=-y1){
            vol_y = -0.0015625*y*y;
        }else if(y<-y1 && y>=-y2){
            vol_y = 0.025*(y+20.0) - 0.35; 
        }else if (y<-y2 && y>=-y3) {
            vol_y = 0.0015625*(y+26.0)*(y+26.0) - 0.45;
        }
        vol_y *= xy_margin;
        if(vol_y >= 0.45*xy_margin){
            vol_y = 0.45*xy_margin;
        }else if(vol_y <= -0.45*xy_margin){
            vol_y = -0.45*xy_margin;
        }
        pwm_y1.write(0.50f +vol_y);
        pwm_y2.write(0.50f -vol_y);
    }else{
        pwm_y1.write(0.50f);
        pwm_y2.write(0.50f);
    }
}

void servo_rot_p(bool& button, int& p, int& temp_angle, Servo6221MG& servo){
    if(button && p >= 0 && p < 180){
        //** up **//
        temp_angle += each_range;
        if(temp_angle >= 180){
            temp_angle = 179;
        }
        while (p < temp_angle) {
            HAL_Delay(25);
            servo.roll(p);
            p += 1;
            led = !led;
            //printf("p:%d\r\n", p);
        }
        printf("now p1: %d\r\n", p);
        HAL_Delay(10);
    }
}

void servo_rot_n(bool& button, int& p, int& temp_angle, Servo6221MG& servo){
    if(button && p > 0 && p < 180){
        //** down **//
        temp_angle -= each_range;
        if(temp_angle < 0){
            temp_angle = 0;
        }
        while (p >= temp_angle) {
            HAL_Delay(25);
            servo.roll(p);
            p -= 1;
            led = !led;
        }
        if(p<0){
            p=0;
        }
        printf("now p1: %d\r\n", p);
        HAL_Delay(10);
    }   
}

void hand_control(bool& b, DigitalOut& valve, int i){
    if(b){
        printf("b_%d pressed!\r\n", i);
        valve = !valve;
        printf("valve%d: %d\r\n",i, valve.read());
        // ここの値を変えて入力の受付時間を調整
        HAL_Delay(300);
    }
}