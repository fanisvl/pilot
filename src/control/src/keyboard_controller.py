import sys
import tty
import termios
from low_level_control import LowLevelController

class KeyboardController:
    def __init__(self, low_level_controller: LowLevelController):
        self.low_level_controller = low_level_controller
        self.steering = 0
        self.throttle = 0
        self.steering_increment = 0.1
        self.throttle_increment = 0.1

    def start(self):
        print("Keyboard Controller started. Use 'i', 'k', 'j', 'l' to control.")
        print("i/k: Throttle, j/l: Steering, 'q' to quit")

        try:
            while True:
                key = self.get_keypress()

                if key == 'i':  # Increase throttle
                    self.increase_throttle()
                elif key == 'k':  # Decrease throttle
                    self.decrease_throttle()
                elif key == 'j':  # Steer left
                    self.steer_left()
                elif key == 'l':  # Steer right
                    self.steer_right()
                elif key == 'q':  # Quit
                    break
        except KeyboardInterrupt:
            pass
        finally:
            self.low_level_controller.shutdown()

    def get_keypress(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def increase_throttle(self):
        self.throttle = min(1, self.throttle + self.throttle_increment)
        self.update_controls()

    def decrease_throttle(self):
        self.throttle = max(-1, self.throttle - self.throttle_increment)
        self.update_controls()

    def steer_left(self):
        self.steering = max(-1, self.steering - self.steering_increment)
        self.update_controls()

    def steer_right(self):
        self.steering = min(1, self.steering + self.steering_increment)
        self.update_controls()

    def update_controls(self):
        print(f"\nThrottle: {self.throttle:.2f}, Steering: {self.steering:.2f}")
        self.low_level_controller.set_steering(self.steering)

def main():
    controller = LowLevelController()
    keyboard_control = KeyboardController(controller)
    try:
        keyboard_control.start()
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()

if __name__ == '__main__':
    main()
