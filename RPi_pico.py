from machine import PWM, Pin
import utime

actuator_relay = Pin(16, Pin.IN)
nucleo_sig = Pin(0, Pin.IN)

reflecter_x = machine.ADC(0)#cds is changed reflecter_x

led_x = Pin(14, Pin.OUT)

ss = PWM(Pin(3, Pin.OUT))#ss = Servo_Seesaw
ss.freq(50)

unit = 0.00005035477#what is this,Fukami? from Takahashi

max_duty = 65025
dig_0 = 0.0725    #0°
dig_90 = 0.12     #90°

nucleo_order = 0#signal from nucleo

while actuator_relay == 0:
    sleep(0.01)#wait until emergency switch turns on


# 初期値
ss.duty_u16(int(max_duty*dig_0))

def read_dis():
    for i in range(100):
        
        voltRaw = cds.read_u16()
        
        print (voltRaw)
        
        if voltRaw > 11000:
            led.value(1)
        else:
            led.value(0)


while True:
    read_dis()
    utime.sleep(0.01)
    
    nucleo_order = nucleo_sig.value()
    if nucleo_order == 1:
        ss.duty_u16(int(max_duty*dig_90))
        utime.sleep(0.01)
    else:
        ss.duty_u16(int(max_duty*dig_0))
        utime.sleep(0.01)
    print(nucleo_order)
###



        
