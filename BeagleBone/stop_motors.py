import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO



Left_Motor_Pin = "P8_19"
Right_Motor_Pin = "P9_14"
Left_Motor_Direction = "P9_26"
Right_Motor_Direction = "P9_25"
GPIO.setup(Left_Motor_Direction, GPIO.OUT)
GPIO.setup(Right_Motor_Direction, GPIO.OUT)
PWM.start(Left_Motor_Pin, 0.0, 20000, 1)                            #Motor 1 (Left)
PWM.start(Right_Motor_Pin, 0.0, 20000, 1)                           #Motor 2 (Right)
PWM.set_duty_cycle(Left_Motor_Pin, 0)                               #initialize PWM
PWM.set_duty_cycle(Right_Motor_Pin, 0)                              

print 'Motors Stopped'