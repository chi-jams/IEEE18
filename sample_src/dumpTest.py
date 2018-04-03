import RPi.GPIO as GPIO

try:
    DUMP_PIN = 8

    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(DUMP_PIN, GPIO.OUT)
    GPIO.output(DUMP_PIN, False)

    input("press any key to dump")

    GPIO.output(DUMP_PIN, True)

except KeyboardInterrupt:
    GPIO.cleanup()
finally:
    GPIO.cleanup()
