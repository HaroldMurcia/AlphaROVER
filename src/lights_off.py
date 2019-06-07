import time, os

os.system("echo 0 > /sys/class/gpio/gpio165/value")      #GPIO 165 LOW
os.system("echo 0 > /sys/class/gpio/gpio166/value")      #GPIO 166 LOW
print "LIGHTS OFF"


