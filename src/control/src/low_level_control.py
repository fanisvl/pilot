import board
from adafruit_pca9685 import PCA9685

class LowLevelController:
    def __init__(self):
        self.i2c = board.I2C()
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 60

        self.NEUTRAL_DUTY = 5900   # 1500 μs pulse (neutral) - 16bit
        self.MAX_LEFT_DUTY = 6700
        self.MAX_RIGHT_DUTY = 5100


        # There's a certain threshold (6240)
        # that the car has to surpass during the initial push for forward movement.
        # Afterwards, lower values result in lower sustained speeds, and the car fully stops at 6100
        # Likewise for reverse.
        self.THROTTLE_NEUTRAL  = 6100 # After throttle initialization, this is stops the vehicle
        self.INIT_FORWARD_DUTY = 6240 # min to start going forward
        self.INIT_REVERSE_DUTY = 5805 # min to start reversing
        print("LowLevelController initialized.")

    def shutdown(self):
        print("shutting down")
        self.pca.deinit()

    def set_steering(self, value):
        if value < -1 or value > 1:
            print("Value must be between -1 and 1.")
            return
        if value == 0:
            duty = self.NEUTRAL_DUTY
        else:
            duty = int(self.NEUTRAL_DUTY + (self.MAX_LEFT_DUTY - self.MAX_RIGHT_DUTY) * (value / 2))
        self.set_duty(1, duty)

    def set_throttle(self, value):

        if value < -1 or value > 1:
            print("Value must be between -1 and 1.")
            return
        if value == 0:
            duty = self.NEUTRAL_DUTY
        elif value > 0:
            duty = int(self.THROTTLE_NEUTRAL + (self.INIT_FORWARD_DUTY - self.THROTTLE_NEUTRAL) * value)
        else:
            duty = self.INIT_REVERSE_DUTY
        self.set_duty(0, duty)
        

    def set_duty(self, channel, duty):
        if duty < 5000 or duty > 6800:
            print(f"duty must be within 5000 and 6800")
            return
        print(f"Setting channel {channel} to duty cycle: {duty}")
        self.pca.channels[channel].duty_cycle = duty

    def calibrate_duty(self, channel):
        duty = self.NEUTRAL_DUTY
        while duty != -1:
            self.set_duty(channel, duty)
            duty = int(input("Enter duty value:"))


def main():
    controller = LowLevelController()
    # controller.calibrate_duty(0)
    # controller.shutdown()

if __name__=='__main__':
    main()
