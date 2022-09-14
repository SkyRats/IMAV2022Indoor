import time
import RPi.GPIO as GPIO

trig = 22 #3 17 Fisico
echo = 12 #26 21 Fisico

trig2 = 0 #31 6 fisico
echo2 = 1 #30 4 fisico

def setup1():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(echo, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(trig, GPIO.OUT)
	GPIO.output(trig, GPIO.LOW)
	time.sleep(1)
	print("setup1 done")

def setup2():
	GPIO.setup(echo2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	GPIO.setup(trig2, GPIO.OUT)
	GPIO.output(trig2, GPIO.LOW)
	time.sleep(1)
	print("setup2 done")

def medir1():
	GPIO.output(trig, GPIO.HIGH)
	time.sleep(0.00001)
	GPIO.output(trig, GPIO.LOW)
	while (GPIO.input(echo) == GPIO.LOW): #and (time.time()-inicio < 1):
		inicio = time.time()
	fim = inicio
	while (GPIO.input(echo) == GPIO.HIGH):
		fim = time.time()
	tempo = fim - inicio
	d = tempo * 17150
	d = round(d + 1.15, 2)
	return d

def medir2():
	GPIO.output(trig2, GPIO.HIGH)
	time.sleep(0.00001)
	GPIO.output(trig2, GPIO.LOW)
	while (GPIO.input(echo2) == GPIO.LOW):
		inicio = time.time()
	fim = inicio
	while (GPIO.input(echo2) == GPIO.HIGH):
		fim = time.time()
	tempo = fim - inicio
	d = tempo*17150
	d = round(d + 1.15,2)
	return d


setup1()
setup2()
while True:
	print("MEDIDAS: (1,2)")
	print(medir1())
	print(medir2())
	time.sleep(0.5)

