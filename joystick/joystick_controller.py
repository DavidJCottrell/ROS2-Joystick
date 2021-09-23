import joystick # type: ignore

import inputs # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from inputs import InputEvent, GamePad, UnknownEventCode # type: ignore

EVENT_ABB = (
    # D-PAD, aka HAT
    ('Absolute-ABS_HAT0X', 'HX'),
    ('Absolute-ABS_HAT0Y', 'HY'),

    # Face Buttons
    ('Key-BTN_NORTH', 'N'),
    ('Key-BTN_EAST', 'E'),
    ('Key-BTN_SOUTH', 'S'),
    ('Key-BTN_WEST', 'W'),

    # Other buttons
    ('Key-BTN_THUMBL', 'THL'),
    ('Key-BTN_THUMBR', 'THR'),
    ('Key-BTN_TL', 'TL'),
    ('Key-BTN_TR', 'TR'),
    ('Key-BTN_TL2', 'TL2'),
    ('Key-BTN_TR2', 'TR3'),
    ('Key-BTN_MODE', 'M'),
    ('Key-BTN_START', 'ST'),

    # PiHUT SNES style controller buttons
    ('Key-BTN_TRIGGER', 'N'),
    ('Key-BTN_THUMB', 'E'),
    ('Key-BTN_THUMB2', 'S'),
    ('Key-BTN_TOP', 'W'),
    ('Key-BTN_BASE3', 'SL'),
    ('Key-BTN_BASE4', 'ST'),
    ('Key-BTN_TOP2', 'TL'),
    ('Key-BTN_PINKIE', 'TR')
)


class JoystickController(object):

    # Default Values
    THRESHOLD: float = 0.5
    ACCURACY: float = 0.2
    TRIGGER_BUTTON: str = "BTN_TRIGGER"

    def __init__(self, publisher: 'JoystickNode', trigger_btn: str, threshold: float, accuracy: float, gamepad=None, abbrevs=EVENT_ABB):
        self.btn_state = {}
        self.old_btn_state = {}
        self.abs_state = {}
        self.old_abs_state = {}
        self.abbrevs = dict(abbrevs)

        self.joystick_publisher = publisher

        self.trigger_held = False

        # Overide default values if they were specified when the node was started
        if trigger_btn: 
            self.TRIGGER_BUTTON = trigger_btn
            print("Joystick trigger set as button " + str(self.TRIGGER_BUTTON))
        else: 
            print("Joystick trigger button not specified. Defaulting to button " + str(self.TRIGGER_BUTTON))

        if threshold: 
            self.THRESHOLD = threshold
            print("Joystick threshold set as " + str(self.THRESHOLD))
        else: print("Joystick threshold not specified. Defaulting to " + str(self.THRESHOLD))

        if accuracy: 
            self.ACCURACY = accuracy
            print("Joystick accuracy set as " + str(self.ACCURACY))
        else: 
            print("Joystick accuracy not specified. Defaulting to " + str(self.ACCURACY))

        # Set default x and y to joystick default resting position values
        self.x: int = 0
        self.y: int = 0

        # Prevents the node from publishing multiple of the same message
        self.last_published_axis: str = "none"

        for key, value in self.abbrevs.items():
            if key.startswith('Absolute'):
                self.abs_state[value] = 0
                self.old_abs_state[value] = 0
            if key.startswith('Key'):
                self.btn_state[value] = 0
                self.old_btn_state[value] = 0
        
        self._other = 0
        
        self.gamepad: GamePad = gamepad
        if not gamepad:
            self._get_gamepad()
    
    def _get_gamepad(self):
        try:
            self.gamepad: GamePad = inputs.devices.gamepads[0]
        except IndexError:
            raise inputs.UnpluggedError("No joystick found.")

    def trigger_down(self): self.trigger_held = True
    
    def trigger_up(self):
        msg: Twist = Twist()
        (msg.linear.y, msg.linear.x) = (0.0, 0.0)
        self.joystick_publisher.publish_demand(msg)
        
        self.trigger_held = False

    def axis_move(self, value: float, axis: str):

        msg = Twist()

        if value > 0:
            # Get x and y values
            value = round((value/1000 / 8) - 1, 2) # Squash between -1 and 1
            if axis == 'ABS_X': self.x = value
            if axis == 'ABS_Y': self.y = value

        # Find which axis is being used
        if self.trigger_held:
            
            # if X Axis
            if abs(self.x) > self.THRESHOLD and abs(self.y) < self.ACCURACY:
                if self.last_published_axis != "x":
                    if self.x < 0: msg.linear.x = -1.0
                    else: msg.linear.x = 1.0
                    self.joystick_publisher.publish_demand(msg)
                self.last_published_axis = "x"

            # if Y Axis
            elif abs(self.y) > self.THRESHOLD and abs(self.x) < self.ACCURACY:
                if self.last_published_axis != "y":
                    if self.y > 0: msg.linear.y = 1.0
                    else: msg.linear.y = -1.0
                    self.joystick_publisher.publish_demand(msg)
                self.last_published_axis = "y"

            # if returned to resting position
            elif (abs(self.x) < self.THRESHOLD and abs(self.y) < self.THRESHOLD):
                if self.last_published_axis != "none":
                    (msg.linear.y, msg.linear.x) = (0.0, 0.0)
                    self.joystick_publisher.publish_demand(msg)
                self.last_published_axis = "none"


    def process_event(self, event: InputEvent):
        if event.ev_type == 'Sync': return
        if event.ev_type == 'Misc': return
        key = event.ev_type + '-' + event.code
        try:
            abbv = self.abbrevs[key]
        except KeyError:
            abbv = self.handle_unknown_event(event, key)
            if not abbv:
                return
        # Button
        if event.ev_type == 'Key' and event.code == self.TRIGGER_BUTTON:
            if self.btn_state[abbv] == 1: self.trigger_up()
            else: self.trigger_down()

            self.old_btn_state[abbv] = self.btn_state[abbv]
            self.btn_state[abbv] = event.state
        # Axis Movement
        if event.ev_type == 'Absolute' and (event.code == "ABS_Y" or event.code == "ABS_X"):
            self.axis_move(self.abs_state[abbv], event.code)

            self.old_abs_state[abbv] = self.abs_state[abbv]
            self.abs_state[abbv] = event.state


    def handle_unknown_event(self, event: InputEvent, key: str):
        """Deal with unknown events."""
        if event.ev_type == 'Key':
            new_abbv = 'B' + str(self._other)
            self.btn_state[new_abbv] = 0
            self.old_btn_state[new_abbv] = 0
        elif event.ev_type == 'Absolute':
            new_abbv = 'A' + str(self._other)
            self.abs_state[new_abbv] = 0
            self.old_abs_state[new_abbv] = 0
        else:
            return None

        self.abbrevs[key] = new_abbv
        self._other += 1

        return self.abbrevs[key]

    def process_events(self):
        try:
            events = self.gamepad.read()
        except (EOFError, UnknownEventCode) as e:
            print(e)
            events = []
        for event in events:
            self.process_event(event)
