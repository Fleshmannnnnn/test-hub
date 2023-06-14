import sensor, time, image,lcd,math
from pyb import Servo,UART,Pin,Timer
from pid import PID
class CARSTATE(object) :
	enSTOP	  =0
	enRUN	   =1
	enBACK	  =2
	enLEFT	  =3
	enRIGHT	 =4
	enRELEASE   =5
	enCATCH	 =6
	enMANUAL	=7
	enTRACING   =8
	enTRAKING   =9
	enAVOIDING  =10
g_carstate = CARSTATE.enTRAKING
g_cardir   = CARSTATE.enSTOP
power	  = 800
ball_s	 =0
global img,key_state,key_event,bluetooth_stutas,THRESHOLD
uart1 = UART(1, 9600)
bluetooth_stutas=0
G_FLAG=0
key_state=0
key_event=0
key = Pin('D8', Pin.IN, Pin.PULL_UP)
THRESHOLD  =[ (53, 99, -13, 46, 29, 57)]
tim = Timer(2, freq=50)
claw=tim.channel(3, Timer.PWM, pin=Pin("A2"), pulse_width_percent=7.5)
global claw_offset
claw_offset=(-20)
claw_release=70
claw_catch=170
count=0
flag_lost=0
pan_servo=Servo(1)
tilt_servo=Servo(2)
pan_angle=90
titl_angle=90
pan_pid  = PID(p=0.1, i=0.01, imax=90)
tilt_pid = PID(p=0.5, i=0.05, imax=90)
dis_pid  = PID(p=0.5, i=0.01)
len_pid  = PID(p=0.2 ,i=0.01)
pwmA1 = Pin('B0')
pwmA2 = Pin('B1')
pwmB1 = Pin('B4')
pwmB2 = Pin('B5')
tim = Timer(3, freq=1000)
Left1 = tim.channel(4, Timer.PWM, pin=pwmA1,pulse_width_percent=0)
Left2 = tim.channel(3, Timer.PWM, pin=pwmA2,pulse_width_percent=0)
Ritht1 = tim.channel(1, Timer.PWM, pin=pwmB1,pulse_width_percent=0)
Ritht2 = tim.channel(2, Timer.PWM, pin=pwmB2,pulse_width_percent=0)
def claw_angle(servo_angle):
	if servo_angle<=0:
		servo_angle=0
	if servo_angle>=180:
		servo_angle=180
	percent=(servo_angle+45)/18
	claw.pulse_width_percent(percent)
def run(left_speed, right_speed):
	if left_speed < -100:
		left_speed = -100
	elif left_speed > 100:
		left_speed = 100
	if right_speed < -100:
		right_speed = -100
	elif right_speed > 100:
		right_speed = 100
	if left_speed > 0:
		Left1.pulse_width_percent(abs(left_speed))
		Left2.pulse_width_percent(abs(0))
	else:
		Left1.pulse_width_percent(abs(0))
		Left2.pulse_width_percent(abs(left_speed))
	if right_speed > 0:
		Ritht1.pulse_width_percent(abs(right_speed))
		Ritht2.pulse_width_percent(abs(0))
	else:
		Ritht1.pulse_width_percent(abs(0))
		Ritht2.pulse_width_percent(abs(right_speed))
def servo_move(servo_n,start_angle,end_angle):
	if start_angle>=end_angle :
		while (start_angle>=end_angle) :
			servo_n.angle(start_angle-2)
			time.sleep(10)
			start_angle=start_angle-2
	return 0
	if start_angle<=end_angle :
		while (start_angle<=end_angle) :
			servo_n.angle(start_angle+2)
			time.sleep(10)
			start_angle=start_angle+2
	return 0
def find_max(blobs):
	max_size=0
	for blob in blobs:
		if blob[2]*blob[3] > max_size:
			max_blob=blob
			max_size = blob[2]*blob[3]
	return max_blob
def color_detect(img):
	global blobs,last_blobs,count,flag_lost
	ball_s=0
	if last_blobs:
		for b in blobs:
			x1 = b[0]-7
			y1 = b[1]-7
			w1 = b[2]+12
			h1 = b[3]+12
			roi2 = (x1,y1,w1,h1)
		blobs = img.find_blobs(THRESHOLD,roi = roi2,area_threshold=100)
		last_blobs = blobs
	else:
		blobs = img.find_blobs(THRESHOLD, area_threshold=100)
		last_blobs = blobs
	if blobs:
		max_blob = find_max(blobs)
		ball_s=10500/((max_blob[2]*2))
		img.draw_rectangle(max_blob.rect())
		img.draw_cross(max_blob.cx(), max_blob.cy())
		img.draw_string(max_blob.cx(), max_blob.cy(), "%.2f mm"%(ball_s))
		pan_error=0
		tilt_output=0
		title_angle=0
		flag_lost=0
		pan_error = img.width()/2-max_blob.cx()
		tilt_error = (img.height()/2-claw_offset)-max_blob.cy()
		tilt_output=tilt_pid.get_pid(tilt_error,1)
		title_angle=tilt_servo.angle()-tilt_output
		tilt_servo.angle(title_angle)
		print("title_angle: ", title_angle)
		if(ball_s>=60 and ball_s<=100) or (title_angle>=70):
			run(0,0)
			if 1:
				count=count+1
				if count >=3:
					count=0
					claw_angle(claw_catch)
					time.sleep(1000)
					servo_move(tilt_servo,tilt_servo.angle(),0)
					time.sleep(1000)
					pan_servo.angle(0)
					time.sleep(1000)
					pan_servo.angle(-100)
					time.sleep(1000)
					claw_angle(claw_release)
					time.sleep(1000)
					pan_servo.angle(0)
					time.sleep(1000)
					pan_servo.angle(90)
					run(-50,-50)
					time.sleep(1000)
					run(0,0)
					tilt_servo.angle(30)
		if(ball_s>100) and (ball_s<1000) and (title_angle<70):
			count=0
			claw_angle(claw_release)
			power=0
			power_s=0
			power_l=0
			if ball_s>300 and ball_s<=2000:
				dis_error=ball_s-300
				power_s=int(dis_pid.get_pid(dis_error,1)/2)
				if(power_s>50):
					power_s=50
				if(power_s<25):
					power_s=25
			elif ball_s>100 and ball_s<=300:
				dis_error=ball_s-100
				power_s=int(dis_pid.get_pid(dis_error,1)/2)
				if(power_s<25):
					power_s=25
			else:
				power_s = 0
			power_l=int(len_pid.get_pid(pan_error,1))
			print("power_s:",power_s)
			print("power_l:",power_l)
			run(power_s-power_l,power_s+power_l)
	else:
		run(0,0)
		claw_angle(claw_release)
		flag_lost=flag_lost+1
		if flag_lost>=5:
			flag_lost=0
			if(tilt_servo.angle()<=30):
				tilt_servo.angle(30)
			else :
				tilt_servo.angle(tilt_servo.angle())
def bluetooth_deal(img):
	global bluetooth_stutas,G_FLAG,g_cardir,g_carstate
	if uart1.any():
		rec=uart1.read(1)
		if rec != None:
			rec=bytes(rec)
			print(rec)
			if rec==b'\x00':
				g_cardir=CARSTATE.enSTOP
			elif rec==b'\x01':
				g_cardir=CARSTATE.enRUN
			elif rec==b'\x02':
				g_cardir=CARSTATE.enBACK
			elif rec==b'\x03':
				g_cardir=CARSTATE.enLEFT
			elif rec==b'\x04':
				g_cardir=CARSTATE.enRIGHT
			elif rec==b'\x05':
				g_carstate=CARSTATE.enTRACING
			elif rec==b'\x06':
				g_carstate=CARSTATE.enTRAKING
			elif rec==b'\x07':
				bluetooth_stutas=bluetooth_stutas+1
				if bluetooth_stutas==1:
					g_carstate=CARSTATE.enAVOIDING
					G_FLAG=1
				if bluetooth_stutas==2:
					g_carstate=CARSTATE.enMANUAL
					G_FLAG=0
					bluetooth_stutas=0
			elif rec==b'\x08':
				g_cardir=CARSTATE.enRELEASE
			elif rec==b'\x09':
				g_cardir=CARSTATE.enCATCH
			elif rec==b'\xF0':
				g_carstate=CARSTATE.enMANUAL
	BST_fLeftMotorOut,BST_fRightMotorOut,g_power=0,0,100
	if (g_carstate==CARSTATE.enMANUAL):
		if(g_cardir==CARSTATE.enSTOP):
			BST_fLeftMotorOut   =0
			BST_fRightMotorOut  =0
		elif(g_cardir==CARSTATE.enRUN):
			BST_fLeftMotorOut  =g_power
			BST_fRightMotorOut =g_power
		elif(g_cardir==CARSTATE.enBACK):
			BST_fLeftMotorOut  =-g_power
			BST_fRightMotorOut =-g_power
		elif(g_cardir==CARSTATE.enLEFT):
			BST_fLeftMotorOut  =-g_power/2
			BST_fRightMotorOut =g_power/2
		elif(g_cardir==CARSTATE.enRIGHT):
			BST_fLeftMotorOut  =g_power/2
			BST_fRightMotorOut =-g_power/2
		run(BST_fLeftMotorOut,BST_fRightMotorOut)
	elif (g_carstate==CARSTATE.enAVOIDING):
		if(g_cardir==CARSTATE.enRUN):
			tilt_servo.angle(tilt_servo.angle()-4)
		elif(g_cardir==CARSTATE.enBACK):
			tilt_servo.angle(tilt_servo.angle()+4)
		elif(g_cardir==CARSTATE.enLEFT):
			pan_servo.angle(pan_servo.angle()+4)
		elif(g_cardir==CARSTATE.enRIGHT):
			pan_servo.angle(pan_servo.angle()-4)
		elif(g_cardir==CARSTATE.enRELEASE):
			claw_angle(claw_release)
		elif(g_cardir==CARSTATE.enCATCH):
			claw_angle(claw_catch)
	elif (g_carstate==CARSTATE.enTRAKING):
		color_detect(img)
claw_angle(claw_release)
tilt_servo.angle(30)
pan_servo.angle(90)
lcd.init()
lcd.clear()
lcd.set_direction(2)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.HQVGA)
sensor.skip_frames(10)
sensor.set_auto_whitebal(False)
clock = time.clock()
blobs = list()
last_blobs = blobs
while True:
	img = sensor.snapshot()
	bluetooth_deal(img)
	lcd.display(img)