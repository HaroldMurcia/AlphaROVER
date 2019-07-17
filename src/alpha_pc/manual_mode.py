import time, os

os.system("echo 1 > /sys/class/gpio/gpio165/value") #GPIO 165 HIGH
os.system("echo 1 > /sys/class/gpio/gpio166/value") #GPIO 166 HIGH
time.sleep(0.3)

os.system("echo 0 > /sys/class/gpio/gpio165/value") #GPIO 165 LOW
os.system("echo 0 > /sys/class/gpio/gpio166/value") #GPIO 166 LOW
time.sleep(0.3)

os.system("echo 1 > /sys/class/gpio/gpio165/value") #GPIO 165 HIGH
os.system("echo 1 > /sys/class/gpio/gpio166/value") #GPIO 166 HIGH
time.sleep(0.3)

os.system("echo 0 > /sys/class/gpio/gpio165/value") #GPIO 165 LOW
os.system("echo 0 > /sys/class/gpio/gpio166/value") #GPIO 166 LOW
time.sleep(0.3)
