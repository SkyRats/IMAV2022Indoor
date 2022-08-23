import os
import subprocess
import binascii

def debug():
	print("Raw data:")
	rawData = (subprocess.Popen("i2cget -y 0 0x5a 0x03 w;", shell=True, stdout=subprocess.PIPE).stdout).read().decode()
	print(rawData)
	print("Env data:")
	envData = (subprocess.Popen("i2cget -y 0 0x5a 0x02 w;", shell=True, stdout=subprocess.PIPE).stdout).read().decode()
	print(envData)

def printaValorCO2():
	envData = (subprocess.Popen("i2cget -y 0 0x5a 0x02 w;", shell=True, stdout=subprocess.PIPE).stdout).read().decode()
	envData_cut = envData[4] + envData[5] + envData[2] + envData[3] #usar bits na ordem 4,5 e 2,3 pra formar o HEX na ordem certa do valor de ppm
	print(int(envData_cut, 16))
	
def retornaValorCO2():
	envData = (subprocess.Popen("i2cget -y 0 0x5a 0x02 w;", shell=True, stdout=subprocess.PIPE).stdout).read().decode()
	envData_cut = envData[4] + envData[5] + envData[2] + envData[3] #usar bits na ordem 4,5 e 2,3 pra formar o HEX na ordem certa do valor de ppm
	return (int(envData_cut, 16))	
	
def start():
	os.system("./start.sh")

if __name__=="__main__":
	#debug()
	#retornaValorCO2()
	printaValorCO2()
