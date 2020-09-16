import time, os

os.system("echo 1 > /sys/class/gpio/gpio166/value")      #GPIO 166 HIGH
os.system("echo 1 > /sys/class/gpio/gpio164/value")      #GPIO 164 HIGH
#print "LASER ON"
