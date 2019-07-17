#! /usr/bin/env python

import smtplib
import subprocess

output=subprocess.check_output(["ifconfig", "eth0"])
output=output.decode('utf-8')
lines=output.split("\n")
ip=lines[1].strip().split("  ")[0].split(":")[1]
#print "Jetson IP: " + str(ip)

fromaddr = 'example@gmail.com'
toaddrs  = 'example@gmail.com'
msg = "Jetson IP [{}]".format(ip) 
 
# Datos
username = 'example@gmail.com'
password =  'examplePSS'
 
# Enviando el correo
server = smtplib.SMTP('smtp.gmail.com:587')
server.starttls()
server.login(username,password)
server.sendmail(fromaddr, toaddrs, msg)
server.quit()
