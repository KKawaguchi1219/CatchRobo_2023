# 鳥取大学ロボットラボラトリー, CatchRobo-2023
## Robot Name: せいんと1号(仮)
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
```cpp
float target_count = 0.0f;
float target_transition = 0.0f;
#define command 25400.0f
float target_plus = command;
float target_minus = -command;
float t_base = command*0.1386962552, t1 = t_base-500, t2 = t_base+500;
//a = (7.21*t1-command)/(t1-t2)*(t1-t2);
float a = fabs((7.21*t1-command)*0.000001);
int target_flag = 0;

Ticker flipper;
#define time 1  // time [ms]

Timer t;
int tim=0;

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
    /*中略*/

    // pid制御用割込み関数
    flipper.attach(&motor_control, chrono::milliseconds(time));

    /*中略*/
}
```
  - 前述のPID制御部分を使った位置制御.
  - 1 msごとのタイマー割込み関数で制御.
  - Timerを使って決められたボタンを押したときから時間を測り, 以下に示す決められた時間区間でPIDに渡す指令値を変える.
    
    - $\rm{command}$ : 必要な回転分のエンコーダの値.  
    - $t$ $\rm{[ms]}$ : 指令値を時間変化させるための独立変数. mbedのTimerにより生成.   
    - $y=y(t)$ : 上記コード内のtarget_transition.  
    - $a$ : 近似二次関数の二次部分の係数. 求め方はコード内を参照. 極大値とその他1点を通る2次関数の決定手法により求める.  
    - $t_1, t_2$  $(t_1 < t_2)$ $\rm{[ms]}$ : 1次関数区間, 2次関数区間, 定数関数区間を区切る場合分け.  
        - 近似した1次関数と $y=\rm{command}$ が交わる $t_{base}$ を求め, そこから $\rm{500}$ $\rm{ms}$ 引いたものを $t_1$ , $\rm{500}$ $\rm{ms}$ 足したものを $t_2$ とした.
        - なぜ500 msかと言うと関数を設計しているときになんかいい感じに見えたので...:shipit:

~~~math
y(t) = 
\begin{cases}
\pm 7.21t & (0 \le t \le t_1)\\
\pm a(t-t_2)^2 \pm \rm{command}  & (t_1 < t \le t_2)\\
\pm \rm{command} & (t > t_2)
\end{cases}
~~~

- 上記の値は以下の手順で決定した.  
1. 必要なエンコーダの値(上記コード上のcommand)をなんとかして読み取る.
2. その値をステップ入力としてPID制御を組み, PIDの各パラメータを調整. 
3. 調整済みのパラメータで指令値までの応答を関数として読み取り, それを1次関数と2次関数で近似.
4. 近似した関数を指令値を変化させる関数として実装.
5. $\rm{command}$ の値をちょっとだけ変えたときに自動で近似関数を調整してくれるように細工.  

#### ハンド移動部
```cpp
#define JYOY_L_CENTER 33
#define JYOY_R_CENTER 17

//x軸モータ: TIM2
PwmOut pwm_x1(PB_10);
PwmOut pwm_x2(PB_3);
float vol_x=0.0f;
//y軸モータ: TIM3
PwmOut pwm_y1(PB_4);
PwmOut pwm_y2(PB_5);
float vol_y=0.0f;

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
```
  - ジョイスティックの傾きに応じてDCモータの回転速度を変えてハンドの位置を制御する.
  - 愚直に変換式実装したりリミットスイッチの制御入れたらくそ長になっちゃった...
  - てか, ここ関数の引数にpwmのインスタンスも含めるべきですね...
  - JYOY_R_CENTERやらJYOY_L_CENTERといった値は, ジョイスティックを触っていないときに出力値(vol_x, vol_y)を0に合わせるためのもの.
  - y_move内のyに2がかかっているのは, 変換式を共通にするための処置.
      - j_LXが-26 ~ +26で変化するのに対して, j_RYは-13 ~ +13の範囲で変化するため.
  - 0で不連続になっているのは, こうした方がジョイスティックを触っていないときに安定してブレーキ状態になるためである.
  > なぜここがフィードバック制御じゃないかというと, 単なる技術不足と怠慢です:trollface:

#### サーボ制御部
```cpp
Servo6221MG servo1(PB_6);
int each_range = 90;
int p1=90;
int temp_angle_1=90;

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

```
- サーボモータ制御部.
- サーボモータは共通エリアのワークを取るためにハンドを90度回転させるのに使った.
- pを1ずつ変化させ, サーボにp度回るように指令を送る.
- 回転速度は servo.roll(p) の1行上の HAL_Delay() で制御.
- Servo6221MG.hは昔の先輩が作ったライブラリをちょっと改造したもの.

#### ハンド制御部
```cpp
DigitalOut valve1(PC_3);
DigitalOut valve2(PC_2);
DigitalOut valve3(PB_7);

void hand_control(bool& b, DigitalOut& valve, int i){
    if(b){
        printf("b_%d pressed!\r\n", i);
        valve = !valve;
        printf("valve%d: %d\r\n",i, valve.read());
        // ここの値を変えて入力の受付時間を調整
        HAL_Delay(300);
    }
}
```
- ハンド制御部
- 今回は空気圧を利用した電磁弁のOnOffでハンドを制御したのでくっっっそ簡単な実装.
- ほとんどLチカなので言うことがない.

## 制御の反省点
- ハンド移動をグリッド制御にして, ワークの位置に来たら自動でいい感じに止まってくれるようにしたらもっと面白かったかもしれない.
- 回転と移動速度がおっっっそい. もうめっちゃ遅い. DCモータのギア比をもっと小さくすべきでした.
- PIDのパラメータの決定に手間取りすぎた. どうやら調べてみると, 世の中にはVRFTやらFRITといったPIDパラメータのオートチューニング手法があるらしいので勉強したみ...
- **TIMが...使えねえ!!!!!!(ドン!!)** mbed側がマスクしすぎて8個あるTIMのうち4個しか使えなかった. あとピンアサインの相談をじっくりしなかったせいで貴重なPWMのピンをただのDigitalOutに消費してしまった.
- 強い人たちの制御と見比べるとやっぱ見劣りする. なにFMT*で経路の自動生成って...てかなんで当たり前にROSとか使えてるの...

# 最後に
ここに置いたコード, ライブラリが誰かの役に立つことを願う. (こういう文言めっちゃ書いてみたかった)
