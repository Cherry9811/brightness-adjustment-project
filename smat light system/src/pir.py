from machine import Pin
import time
from machine import Pin

pir = Pin(4, Pin.IN, Pin.PULL_DOWN)
 
print('Starting up the PIR Module')
time.sleep(1)
print('Ready')
yellow =Pin(13, Pin.OUT)


while True:
    if pir.value() == 1:
        print("Motion detected")
        yellow.on()

    else:
        print("No motion detected")
        yellow.off()

    time.sleep(1)

     
