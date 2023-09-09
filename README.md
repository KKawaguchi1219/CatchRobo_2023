# 鳥取大学ロボットラボラトリー, CatchRobo-2023, せいんと1号(仮)
Robot control code for CatchRobo-2023

### 開発ブランチ
exp_branch

### 開発環境, ファイル概要
- 開発環境：keil studio cloud(mbed-os), Thonny(MicroPython)  
- 開発ボード：Nucleo F446re, Raspberry-pi Pico  
- mbed ver：mbed-os 6.15.1  
- main.cpp：source code file  
- mbed_app.json：command file for floating point display(mbed)  
- Encoder_InterruptIn：library files for reading encoder value  
- WII_CLASSIC_CONTROLLER_：library files for Wii Classic Controller  
- Servo6221MG：library files for jx-servo6221MG (notice: code is ajusted for center-aligned pwm)  

### 主要なプログラム説明
#### PID制御部(DCモータの位置制御用)
~~~ cpp
float diff[2];
#define Kp 0.0245f           // pゲイン 
#define Ki 0.09f             // iゲイン     
#define Kd 0.0003f           // dゲイン   
#define Krc 0.30f            // 指数平均ゲイン
#define limit_duty 0.44f
static float pre_i = 0.0f;

float pid_motor(float count, float target){
    float p, i, d;
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
~~~
  - よく見る感じのPID制御.
  - 今回はサーボ制御であり, 決められた範囲の値(-0.50 ~ +0.50)を出力させる必要があった.
  - 低回転速度になるとエンコーダの値が安定しないため, D項の計算に指数平均を実装した.
  - 応答性を高めるために固定アンチワインドアップを実装した.

#### 回転(位置)制御部
~~~ cpp
Ticker flipper;
#define time 1  // time [ms]

Timer t;
int tim=0;

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
    vol_rot = (pid_motor(encoder_count, target_transition));
    pwm_rot1.write(0.50f -vol_rot);
    pwm_rot2.write(0.50f +vol_rot);
}

int main(void){
    /*中略*/

    // pid制御用割込み関数
    flipper.attach(&motor_control, chrono::milliseconds(time));

    /*中略*/
}
~~~
  - 前述のPID制御部分を使った位置制御.
  - 1 msごとのタイマー割込み関数で制御している.
  - Timerを使って決められたボタンを押したときから時間を測り, 以下に示す決められた時間区間でPIDに渡す指令値を変える.
~~~math
y = 
\begin{cases}
\pm 7.21t & (0 \le t \le 3000)\\
\pm 0.00362(t-4000)^2 \pm 25250  & (3000 < t \le 4000)\\
\pm 25250 & (t > 4000)
\end{cases}
~~~
  - 上記の値は以下の手法で決定した.  
    1. 必要なエンコーダの値を読み取る.
    2. その値をステップ入力としてPID制御を組み, PIDの各パラメータを調整. 
    3. 調整済みのパラメータで指令値までの応答を関数として読み取り, それを1次関数と2次関数で近似.
    4. 近似した関数を指令値を変化させる関数として実装

#### ハンド移動部
~~~ cpp
void x_move(unsigned char& j_LX){
    float x = j_LX - JYOY_L_CENTER-2;
    float x1 = 8.0f;
    float x2 = 18.0f;
    float x3 = 26.0f;

    if(x != 0.0f){
            // 変換式
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

        if(vol_x >= 0.45){
            vol_x = 0.45;
        }else if(vol_x <= -0.45){
            vol_x = -0.45;
        }
        pwm_x1.write(0.50 -vol_x);
        pwm_x2.write(0.50 +vol_x);
    }else{
        pwm_x1.write(0.50f);
        pwm_x2.write(0.50f);
    }
}

void y_move(unsigned char& j_RY){
    float y = 2.0*(j_RY - JYOY_R_CENTER-1);
    float y1 = 8.0f;
    float y2 = 18.0f;
    float y3 = 26.0f;
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
        if(vol_y >= 0.45){
            vol_y = 0.45;
        }else if(vol_y <= -0.45){
            vol_y = -0.45;
        }
        pwm_y1.write(0.50f -vol_y);
        pwm_y2.write(0.50f +vol_y);
    }else{
        pwm_y1.write(0.50f);
        pwm_y2.write(0.50f);
    }
}
~~~
  - ジョイスティックの傾きに応じてDCモータの回転速度を変えてハンドの位置を制御する.
  - 今回は直交した動きだったのでバカみたいな座標変換の必要もなく, 適当な自作の関数に合わせているだけである.
  - JYOY_R_CENTERやらJYOY_L_CENTERといった値は, ジョイスティックを触っていないときに出力値(vol_x, vol_y)を0に合わせるためのもの.
  - y_move内のyに2がかかっているのは, 変換式を共通にするための処置.
      - j_LXが-26 ~ +26で変化するのに対して, j_RYは-13 ~ +13の範囲で変化するため.
  - 0で不連続になっているのは, こうした方がジョイスティックを触っていないときに安定してブレーキ状態になるためである.
  - なぜここがフィードバック制御じゃないかというと, 単なる技術不足と怠慢です:trollface:

