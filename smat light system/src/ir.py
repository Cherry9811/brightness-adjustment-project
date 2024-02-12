from machine import Pin, PWM, Timer
import utime
import time

led_pin = 16
led2_pin = 17
led3_pin = 18

led = PWM(Pin(led_pin), freq=1000)
led2 = PWM(Pin(led2_pin), freq=1000)
led3 = PWM(Pin(led3_pin), freq=1000)

sensor_pin = 15
sensor2_pin = 14
sensor3_pin = 13

sensor = Pin(sensor_pin, Pin.IN)
sensor2 = Pin(sensor2_pin, Pin.IN)
sensor3 = Pin(sensor3_pin, Pin.IN)


button_pin = 12
button = Pin(button_pin, Pin.IN, Pin.PULL_UP)


low_brightness = 100
high_brightness = 1023

button_timer = Timer()


while True:
    motion_ir1 = sensor.value() == 0
    motion_ir2 = sensor2.value() == 0
    motion_ir3 = sensor3.value() == 0

    button_pressed = not button.value()

    if motion_ir1:
        led.duty_u16(high_brightness)
        led2.duty_u16(high_brightness)
        led3.duty_u16(low_brightness)
        utime.sleep(20)
    elif motion_ir2:
        led.duty_u16(low_brightness)
        led2.duty_u16(high_brightness)
        led3.duty_u16(high_brightness)
        utime.sleep(20)
    elif motion_ir3:
        led.duty_u16(low_brightness)
        led2.duty_u16(high_brightness)
        led3.duty_u16(high_brightness)
        utime.sleep(20)
    elif button_pressed:
        print("pressed")
        led.duty_u16(high_brightness)
        led2.duty_u16(high_brightness)
        led3.duty_u16(high_brightness)
        utime.sleep(20)
        led.duty_u16(low_brightness)
        led2.duty_u16(low_brightness)
        led3.duty_u16(low_brightness)
    else:
        led.duty_u16(low_brightness)
        led2.duty_u16(low_brightness)
        led3.duty_u16(low_brightness)
