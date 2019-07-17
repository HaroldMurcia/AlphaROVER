import time, os

os.system("echo 1 > /sys/class/gpio/gpio165/value")      #GPIO 165 HIGH
print "LEDS ON"
