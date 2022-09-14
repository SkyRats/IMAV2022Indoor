import time
import serial
import adafruit_us100
import os
import subprocess
st = time.time()
class CCS:
    def debug(self):
        print("Raw data:")
        rawData = (subprocess.Popen("i2cget -y 0 0x5a 0x03 w;", shell=True, stdout=subprocess.PIPE).stdout).read().decode()
        print(rawData)
        print("Env data:")
        envData = (subprocess.Popen("i2cget -y 0 0x5a 0x02 w;", shell=True, stdout=subprocess.PIPE).stdout).read().decode()
        print(envData)
    def printaValorCO2(self):
        envData = (subprocess.Popen("i2cget -y 0 0x5a 0x02 w;", shell=True, stdout=subprocess.PIPE).stdout).read().decode()
        envData_cut = envData[4] + envData[5] + envData[2] + envData[3] #usar bits na ordem 4,5 e 2,3 pra formar o HEX na ordem certa do valor de ppm
        print(int(envData_cut, 16))
    def retornaValorCO2(self):
        envData = (subprocess.Popen("i2cget -y 0 0x5a 0x02 w;", shell=True, stdout=subprocess.PIPE).stdout).read().decode()
        envData_cut = envData[4] + envData[5] + envData[2] + envData[3] #usar bits na ordem 4,5 e 2,3 pra formar o HEX na ordem certa do valor de ppm
        
        return (int(envData_cut, 16))	

class US100:
    def medicaoUS100(self):
        uart = serial.Serial("/dev/ttySAC0", baudrate=9600, timeout=1)
        #uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
        #uart = serial.Serial("/dev/ttyAML0", baudrate=9600, timeout=1)
        us100 = adafruit_us100.US100(uart)
        #print("-----")
        #print("Temperature: ", us100.temperature)
        time.sleep(0.5)
        distmedida = us100.distance
        temp = us100.temperature
        #print("Distance: ", us100.distance)
        time.sleep(0.5)
        return (distmedida, temp)
class CSVconverter:
    def __init__(self, CO2, temp, Distancia):
        import csv  
        import time
        et = time.time()
        Tempo = et - st
        header = ['CO2', 'Temperatura', 'Umidade', 'Distância', 'Tempo']
        data = [CO2,temp,Distancia,Tempo]
        with open('path to file', 'w', encoding='UTF8') as f: #Coloque o path para um arquivo de texto para salvar os dados
            writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(data)
        return("1")
class Main:
    def Main(self):
        #a = CCS().retornaValorCO2()
        b,c = US100().medicaoUS100()
        print("Distance: ", b)
        print("Temperature: ", c)
        #print("CO2: ", a)
        #z = CSVconverter().__init__(self,a,b,c)
while True:
    #CCS().start()
    Main().Main()
    time.sleep(1)
        
