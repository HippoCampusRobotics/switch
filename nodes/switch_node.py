#!/usr/bin/env python
# https://github.com/jgarff/rpi_ws281x
# python bindings: https://github.com/rpi-ws281x/rpi-ws281x-python
# add user to gpio group
# udev rule:
# KERNEL=="spidev*", RUN="/bin/sh -c 'chgrp -R gpio /dev/spidev* && chmod -R g+rw /dev/spidev*'"
# attention: do not use higher color value than 254!


from rpi_ws281x import PixelStrip, Color
import time
import pigpio

LED_COUNT = 4
LED_PIN = 10
BUZZER_PIN = 18
LED_FREQ_HZ = 900000
LED_DMA = 10
LED_INVERT = False
LED_CHANNEL = 0

pi = pigpio.pi()

def set_tone(tone):
    pi.set_PWM_frequency(BUZZER_PIN, tone)
    pi.set_PWM_dutycycle(BUZZER_PIN, 128)

def switch_buzzer(on):
    if on:
        pi.set_PWM_dutycycle(BUZZER_PIN, 128)
    else:
        pi.set_PWM_dutycycle(BUZZER_PIN, 0)


def buzzer_blip():
    pi.write(BUZZER_PIN, pigpio.HIGH)
    pi.write(BUZZER_PIN, pigpio.LOW)

def colorWipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms / 1000.0)
if __name__ == '__main__':
    switch_buzzer(False)
    strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, 255, LED_CHANNEL)
    strip.begin()
    set_tone(4000)
    switch_buzzer(True)
    try:
        while True:
            buzzer_blip()
            colorWipe(strip, Color(254, 0, 0), 100)
            colorWipe(strip, Color(0, 254, 0), 100)
            colorWipe(strip, Color(0, 0, 254), 100)
    except KeyboardInterrupt:
        colorWipe(strip, Color(0, 0, 0))
        switch_buzzer(0)
