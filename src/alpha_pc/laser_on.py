import time, os

os.system("echo 1 > /sys/class/gpio/gpio166/value")      #GPIO 166 HIGH
print "LASER ON"
