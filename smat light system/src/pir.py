from machine import Pin
import time
 
pir = Pin(4, Pin.IN, Pin.PULL_DOWN)
 
print('Starting up the PIR Module')
time.sleep(1)
print('Ready')
 
while True:
    if pir.value() == 1:
        print("Motion detected")
    else:
        print("No motion detected")

    time.sleep(1)

     
