"""SDL controller input; polled on the Qt thread, with no ROS side effects."""

import os


def empty_input():
    result = dict.fromkeys(
        ('forward', 'reverse', 'left', 'right', 'reset', 'emergency_stop'), False)
    result.update(analog=False, throttle=0.0, brake=0.0,
                  steering_position=False, steering=0.0)
    return result


class GamepadGate:
    """Require neutral controls after every manual start or fault."""

    def __init__(self, deadzone=0.2, trigger_threshold=0.1):
        self.deadzone = deadzone
        self.trigger_threshold = trigger_threshold
        self._reset_down = False
        self.reset()

    def reset(self):
        self.ready = False
        self._dpad_mode = False

    @staticmethod
    def _axis(value, deadzone):
        magnitude = max(0.0, min(1.0, (abs(value) - deadzone) / (1.0 - deadzone)))
        return -magnitude if value < 0 else magnitude

    def neutral(self, sample):
        return (sample is not None
                and abs(sample['steering']) <= self.deadzone
                and sample['forward'] <= self.trigger_threshold
                and sample['reverse'] <= self.trigger_threshold
                and not any(sample[key] for key in ('left', 'right', 'reset', 'stop')))

    def reset_requested(self, sample):
        """One Y request per press, with driving controls released."""
        down = bool(sample is not None and sample['estop_reset'])
        requested = down and not self._reset_down and self.neutral(sample)
        self._reset_down = down
        return requested

    def update(self, sample, active):
        result = empty_input()
        if not active:
            self.reset()
            return result
        if sample is None:
            # Startup has not accepted input yet: wait without latching E-Stop.
            # Losing input after neutral was accepted remains an emergency.
            result['emergency_stop'] = self.ready
            self.reset()
            return result
        if sample['stop']:
            self.reset()
            result['emergency_stop'] = True
            return result
        if not self.ready:
            self.ready = self.neutral(sample) and not sample['estop_reset']
            return result
        steering = self._axis(sample['steering'], self.deadzone)
        # D-pad holds its accumulated angle after release. Moving the stick
        # takes back position control; X also returns to centered position mode.
        if steering or sample['reset']:
            self._dpad_mode = False
        elif sample['left'] or sample['right']:
            self._dpad_mode = True
        result.update(
            analog=True,
            throttle=self._axis(sample['forward'], self.trigger_threshold),
            brake=self._axis(sample['reverse'], self.trigger_threshold),
            steering_position=not self._dpad_mode,
            steering=0.0 if sample['reset'] else steering,
            left=self._dpad_mode and sample['left'],
            right=self._dpad_mode and sample['right'],
            reset=sample['reset'])
        return result


class XboxController:
    """Use SDL's named axes so USB and Bluetooth mappings may differ."""

    def __init__(self):
        self.device = None
        self.pygame = None
        self.controllers = None
        self.status = 'Controller: not connected'

    def read(self):
        if self.pygame is None:
            os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
            # Qt owns the visible window; SDL must still receive input.
            os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')
            try:
                import pygame
                from pygame._sdl2 import controller
            except ImportError:
                self.status = 'Controller unavailable: install python3-pygame'
                return None
            self.pygame = pygame
            self.controllers = controller
        try:
            self.pygame.display.init()
            self.controllers.init()
            self.pygame.event.get()
            if self.device is not None and not self.device.attached():
                self.device.quit()
                self.device = None
                self.status = 'Controller: disconnected'
                return None
            if self.device is None:
                for index in range(self.controllers.get_count()):
                    name = self.controllers.name_forindex(index) or ''
                    if ('xbox' in name.lower()
                            and self.controllers.is_controller(index)):
                        self.device = self.controllers.Controller(index)
                        break
            if self.device is None:
                self.status = 'Controller: no Xbox controller detected'
                return None
            self.status = f'Controller: {self.device.name}'
            pg = self.pygame
            axis = self.device.get_axis
            button = self.device.get_button
            return {
                'steering': axis(pg.CONTROLLER_AXIS_LEFTX) / 32768.0,
                'forward': axis(pg.CONTROLLER_AXIS_TRIGGERRIGHT) / 32768.0,
                'reverse': axis(pg.CONTROLLER_AXIS_TRIGGERLEFT) / 32768.0,
                'left': bool(button(pg.CONTROLLER_BUTTON_DPAD_LEFT)),
                'right': bool(button(pg.CONTROLLER_BUTTON_DPAD_RIGHT)),
                'reset': bool(button(pg.CONTROLLER_BUTTON_X)),
                'stop': bool(button(pg.CONTROLLER_BUTTON_B)),
                'estop_reset': bool(button(pg.CONTROLLER_BUTTON_Y)),
            }
        except self.pygame.error as exc:
            self.status = f'Controller error: {exc}'
            self.close()
            return None

    def close(self):
        if self.device is not None:
            self.device.quit()
            self.device = None
        if self.controllers is not None:
            self.controllers.quit()
        if self.pygame is not None:
            self.pygame.display.quit()
