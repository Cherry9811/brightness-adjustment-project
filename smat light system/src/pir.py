import time
from machine import Pin

pir = Pin('P18',mode=Pin.IN, pull=Pin.PULL_UP)
last_motion = -5

while True:
    if pir() == 1:
        if time.time() - last_motion > 5:
            last_motion = time.time()
            print("Motion detected")
    else:
        last_trigger = 0
        print("No motion detected")

    time.sleep(5)
