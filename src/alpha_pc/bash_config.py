import time, os

#os.system("echo 165 > /sys/class/gpio/export") #Enable GPIO 165 = PIN 55 = LEDS
#os.system("echo out > /sys/class/gpio/gpio165/direction")        #GPIO 165 OUT

#os.system("echo 166 > /sys/class/gpio/export") #Enable GPIO 166 = PIN 58 = LASER
#os.system("echo out > /sys/class/gpio/gpio166/direction")        #GPIO 166 OUT
#time.sleep(0.1)

os.system("echo 1 > /sys/class/gpio/gpio165/value") #GPIO 165 HIGH
os.system("echo 1 > /sys/class/gpio/gpio166/value") #GPIO 166 HIGH
time.sleep(0.1)

os.system("echo 0 > /sys/class/gpio/gpio165/value") #GPIO 165 LOW
os.system("echo 0 > /sys/class/gpio/gpio166/value") #GPIO 166 LOW
time.sleep(0.1)

os.system("echo 1 > /sys/class/gpio/gpio165/value") #GPIO 165 HIGH
os.system("echo 1 > /sys/class/gpio/gpio166/value") #GPIO 166 HIGH
time.sleep(0.1)

os.system("echo 0 > /sys/class/gpio/gpio165/value") #GPIO 165 LOW
os.system("echo 0 > /sys/class/gpio/gpio166/value") #GPIO 166 LOW
time.sleep(0.1)

print "ACCEDIDO"

