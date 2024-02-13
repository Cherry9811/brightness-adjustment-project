from machine import Pin
import utime

# pin numbers
TRIGGER_PIN = 3
ECHO_PIN = 2

# pins
trigger = Pin(TRIGGER_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

def measure_distance():
    # Send trigger signal
    trigger.low()
    utime.sleep_us(2)
    trigger.high()
    utime.sleep_us(5)
    trigger.low()
    
    #Calculate the duration of the echo signal
    while echo.value() == 0:
        signal_off = utime.ticks_us()
    while echo.value() == 1:
        signal_on = utime.ticks_us()
    
    # Calculate distance using the duration
    time_passed = signal_on - signal_off
    distance = (time_passed * 0.0343) / 2
    
    return distance

def main():
    while True:
        distance = measure_distance()
        print("The distance from the object is {:.2f} cm".format(distance))
        utime.sleep(1)

if __name__ == "__main__":
    main()

