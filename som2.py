import time
import RPi.GPIO as GPIO

trig = 22 # 17 Fisico
echo = 12 # 21 Fisico

def setup():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(echo, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(trig, GPIO.OUT)
	GPIO.output(trig, GPIO.LOW)
	time.sleep(2)
	print("setup done")

def medir():
	GPIO.output(trig, GPIO.LOW)
	time.sleep(0.000002)
	GPIO.output(trig, GPIO.HIGH)
	time.sleep(0.00001)
	GPIO.output(trig, GPIO.LOW)
	time.sleep(0.00003)
	inicio = time.time()
	while (GPIO.input(echo) == GPIO.LOW) and (time.time()-inicio< 0.5):
		pass
	inicio = time.time()
	while (GPIO.input(echo) == GPIO.HIGH): #and (time.time()-inicio < 0.5):
		pass
	tempo = time.time() - inicio
	d = tempo * 17150
	d = round(d + 1.15, 2)
	return d

setup()
while True:
	print("MEDINDO")
	time.sleep(0.1)
	print('  '+str(medir()))
