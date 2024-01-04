
import time
import math

from machine import Pin, I2C,PWM,ADC,UART

# ==================================================================================================
uart = machine.UART(0, baudrate=9600,tx=machine.Pin(16),rx=machine.Pin(17)) 
led=Pin(25,Pin.OUT)
i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400000)

M1=PWM(Pin(18))
M1.freq(6000)
M2=PWM(Pin(20))
M2.freq(6000)
M3=PWM(Pin(22))
M3.freq(6000)
dt=float(0.01) 
# =======================================================================================================

MPU6050_ADDR = 0x68

MPU6050_INT_ENABLE = 0x38
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_ACCEL_XOUT_L = 0x3C
MPU6050_ACCEL_YOUT_H = 0x3D
MPU6050_ACCEL_YOUT_L = 0x3E
MPU6050_ACCEL_ZOUT_H = 0x3F
MPU6050_ACCEL_ZOUT_L = 0x40
MPU6050_TEMP_OUT_H = 0x41
MPU6050_TEMP_OUT_L = 0x42
MPU6050_GYRO_XOUT_H = 0x43
MPU6050_GYRO_XOUT_L = 0x44
MPU6050_GYRO_YOUT_H = 0x45
MPU6050_GYRO_YOUT_L = 0x46
MPU6050_GYRO_ZOUT_H = 0x47
MPU6050_GYRO_ZOUT_L = 0x48
MPU6050_PWR_MGMT_1 = 0x6B

MPU6050_LSBC = 340.0
MPU6050_TEMP_OFFSET = 36.53
MPU6050_LSBG = 16384.0
MPU6050_LSBDS = 131.0
# =======================================================================================================
print("yes works")
axcall=0
aycall=0
azcall=0
# Function to blink the LED
prev_time=time.ticks_ms() 
def blink_led(times, interval):
    for _ in range(times):
        led.on()
        time.sleep(interval)
        led.off()
        time.sleep(interval)

# Blink the LED 20 times
blink_led(15, 0.1)


prev_time=float(0.0)
current_time=float(0.0)
Out1=int(0)
Out2=int(0)
Out3=int(0)
ax=float()
ay=0.0
gx=int()
gy=int()
gz=int()
thro=int(0)# THROTILE

PID_PITCH=int(0)#PID OUTPUT PITCH 
PID_ROLL=int(0)# PID OUTPUT ROLL
PID_YAW=int(0)# PID OUTPUT YAW
PID_thro=int(0)# PID OUTPUT THORITILE

e_pitch=float(0)# errot in pitch
e_roll=float(0)# errror in roll
e_thro=float(0)# errror in thro
e_yaw=float(0) # errror in yaw


kp_pitch=float(1)# pitch proportional term
ki_pitch=float(0.0820)# pitch integral term
kd_pitch=float(2)# pitch derivative term


kp_roll=float(1)# roll proportional term
ki_roll=float(0.0820)# roll integrel term
kd_roll=float(2)# roll derivative term



kp_thro=float()# roll proportional term
ki_thro=float()# roll integrel term
kd_thro=float()# roll derivative term



kp_yaw=float(0.0)# yaw proportional term
ki_yaw=float(0.0)# yaw integral term
kd_yaw=float(0.0)# yaw derivative term


# initial values from the  remote

pitchvalue=int(0)
rollvalue=int(0)

yawvalue=int()
# PID variables 
integral_thro=float(0.0)
integral_pitch=float(0.0)
integral_roll=float(0.0)
integral_yaw=float(0.0)
derivative_thro=float(0.0)
derivative_pitch=float(0.0)
derivative_roll=float(0.0)
derivative_yaw=float(0.0)
# previous terms to save new term
integral_pitchpre=float(0)
integral_rollpre=int(0)
integral_yawpre=int(0)
e_yawpre=float(0.0)
E_pitch=0.0
E_roll=0.0
proportional_pitch=float(0.0)
proportional_roll=float(0.0)
pitchin=0
pitchint=0
rollin=0
rollint=0


# ============================================================================================================================

alpha=0.5

pitchin=0
rollin=0

# ==========================================calibertation

# =========================================

def low_pass_filter(preval,newval):
    return alpha*preval+(1-alpha)*newval


def mpu6050_init(i2c):
    i2c.writeto_mem(MPU6050_ADDR, MPU6050_PWR_MGMT_1, bytes([0])) # disable sleep
    i2c.writeto_mem(MPU6050_ADDR, MPU6050_INT_ENABLE, bytes([1])) # enable interrupt on data ready (DATA_READ_EN)


def combine_register_values(h, l):
    if not h[0] & 0x80:
        return h[0] << 8 | l[0]
    return -((h[0] ^ 255) << 8) |  (l[0] ^ 255) + 1


def mpu6050_get_temp(i2c):
    temp_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_TEMP_OUT_H, 1)
    temp_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_TEMP_OUT_L, 1)
    
    return (combine_register_values(temp_h, temp_l) / MPU6050_LSBC) + MPU6050_TEMP_OFFSET


def mpu6050_get_accel(i2c):
    accel_x_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1)
    accel_x_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_L, 1)
    accel_y_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, 1)
    accel_y_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_L, 1)
    accel_z_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, 1)
    accel_z_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_L, 1)
    
    return [combine_register_values(accel_x_h, accel_x_l) / MPU6050_LSBG,
            combine_register_values(accel_y_h, accel_y_l) / MPU6050_LSBG,
            combine_register_values(accel_z_h, accel_z_l) / MPU6050_LSBG]


def mpu6050_get_gyro(i2c):
    gyro_x_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1)
    gyro_x_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_XOUT_L, 1)
    gyro_y_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_YOUT_H, 1)
    gyro_y_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_YOUT_L, 1)
    gyro_z_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, 1)
    gyro_z_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_GYRO_ZOUT_L, 1)
    
    return [combine_register_values(gyro_x_h, gyro_x_l) / MPU6050_LSBDS,
            combine_register_values(gyro_y_h, gyro_y_l) / MPU6050_LSBDS,
           combine_register_values(gyro_z_h, gyro_z_l) / MPU6050_LSBDS]

def read_integer_from_uart():
    data_buffer = b""
    while uart.any():
            data_buffer += uart.read(1)
            if data_buffer.endswith(b"\n"):
                   try:
                    data_str = data_buffer.decode().strip()
             
                    integer_value = int(data_str)
                
                    # Ensure the value is within the range of 1 to 999
                    return (integer_value)
                   except ValueError:
                        pass
    return None


# ======================================================================================

        
        
#=======================================================================================     
    
# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
def acceldata(ax,ay,az):
   global pitchint
   global rollint
   ax=ax-axcall
   ay=ay-aycall
   az=az-azcall 
   rollin=math.atan2(ay,math.sqrt(ax*ax+az*az))*180/math.pi
   pitchin=math.atan2(-ax,math.sqrt(ay*ay+az*az))*180/math.pi
   pitchint=int(low_pass_filter(pitchint,pitchin))
   rollint=int(low_pass_filter(rollint,rollin))  

def pidcontrol(pitchvalue,rollvalue,dt):
    global integral_pitch
    global integral_pitchpre
    global integral_roll
    global integral_rollpre
    global E_pitch
    global E_roll
 
# \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

# -----------------------------------------------------------------------------------------------------------------
#  |               pitch
# -----------------------------------------------------------------------------------------------------------------
  
    e_pitch=(-pitchvalue)-(-1*pitchint)
    proportional_pitch=kp_pitch*e_pitch
    
    integral_pitch=((integral_pitchpre + e_pitch)*ki_pitch)*dt
    if thro<=1:
        integral_pitch=0
   
    derivative_pitch=kd_pitch*(e_pitch-E_pitch)/dt
    
    PID_PITCH=(100*int((proportional_pitch+integral_pitch+derivative_pitch)))
    E_pitch=e_pitch
    integral_pitchpre=integral_pitch

# -----------------------------------------------------------------------------------------------------------------
#  |                roll
# -----------------------------------------------------------------------------------------------------------------
    e_roll=(-rollvalue)-(-1*rollint)
    proportional_roll=kp_roll*e_roll
    
    integral_roll=((integral_rollpre+e_roll)*ki_roll)*dt
    if thro<=1:
       integral_roll=0
    
    derivative_roll=kd_roll*(e_roll-E_roll)/dt
    
    PID_ROLL=(100*int((proportional_roll+integral_roll+derivative_roll)))
    E_roll=e_roll
    integral_rollpre=integral_roll
    return PID_PITCH,PID_ROLL

# --------------------------------------------------------
  
  
    

def motormixer(PID_PITCH,PID_ROLL):
# MAIN MOTOR MIXER FROM THE PID AND REMOTE
  global Out1
  global Out2
  global Out3
  
  O1=int(thro-PID_PITCH)
  O2=int(thro+PID_PITCH+PID_ROLL)
  O3=int(thro+PID_PITCH-PID_ROLL)
  Out1=max(0,min(O1,65530))
  Out2=max(0,min(O2,65530))
  Out3=max(0,min(O3,65530))
  M1.duty_u16(Out1)
  M2.duty_u16(Out2)
  M3.duty_u16(Out3)#*******************************************************************************************************************
#                                    OUTPUT LIMIT PARAMETER
  
    
# *****************************************************************************************************************    
    
def cali(times, interval):
    mpu6050_init(i2c)
    global axcall
    global aycall
    global azcall
    for _ in range(times):
        axx,ayy,azz=mpu6050_get_accel(i2c)

        axcall=axcall+axx
        aycall=aycall+ayy
        azcall=azcall+azz
       
# Blink the LED 20 times
cali(100, 0.1)
axcall=axcall/100
aycall=aycall/100
azcall=azcall/100-0.75
blink_led(1,5)
state=True
while state==True:
    
    current_time=time.ticks_ms()
    dt=(current_time-prev_time)
    prev_time=current_time
    mpu6050_init(i2c)
    data = read_integer_from_uart()
    if data is not None:
#           print("Received integer data:",data)
          if 0<=data<=100:
            thro=data/100
            throvalue=57050
            thro=thro*throvalue
          if 200<=data<=300:
           pitchvalue=(data-200)
           pitchvalue=pitchvalue-50
          if 400<=data<=500:
            rollvalue=(data-400)
            rollvalue=rollvalue-50
    ax,ay,az=mpu6050_get_accel(i2c)          
    acceldata(ax,ay,az)
    PID_PITCH,PID_ROLL=pidcontrol(pitchvalue,rollvalue,dt)
    motormixer(PID_PITCH,PID_ROLL)
#  print(  pitchint)
#     print(pitchint,"\t",PID_PITCH,"\t",rollint,"\t",PID_ROLL)
 #    print("output", Out1,"/t",Out2,"/t",Out3)
#     print("p",pitchint)
#    print("pitchva",pitchvalue,"rollva",rollvalue)
  
    time.sleep_ms(28)
    
    
  
    
    





