import threading
import pygame
import pygame.joystick as joystick
import pygame.event, pygame.time
from pygame.constants import JOYBALLMOTION, JOYBUTTONDOWN, JOYAXISMOTION, JOYHATMOTION

# PS4 BUTTONS
BUTTON_X        = 0
BUTTON_CIRCLE   = 1
BUTTON_TRIANGLE = 2
BUTTON_SQUARE   = 3
BUTTON_L1       = 4
BUTTON_R1       = 5
BUTTON_L2       = 6
BUTTON_R2       = 7
BUTTON_SHARE    = 8
BUTTON_OPTIONS  = 9
BUTTON_PS       = 10
BUTTON_L3       = 11
BUTTON_R3       = 12

# PS4 JOYSTICKS
L3_X = 0
L3_Y = 1
L2_Y = 2
R3_X = 3
R3_Y = 4
R2_Y = 5

class Controller:

    def __init__(self) -> None:
        pygame.init()

        self.j = joystick.Joystick(0)
        self.j.init()

        self.running = True

        self.game_thread = threading.Thread(target=self.game_loop)
        self.game_thread.start()

    def on_button_pressed(self, button_id):
        pass

    def on_joystick_move(self, joystick_id, value):
        pass

    def on_hat_push(self, x, y):
        pass

    def game_loop(self):
        clock = pygame.time.Clock()
        while self.running:
            clock.tick(60)
            for event in pygame.event.get():
                if event.type == JOYBUTTONDOWN:
                    self.on_button_pressed(event.button)
                if event.type == JOYAXISMOTION:
                    self.on_joystick_move(event.axis, event.value)
                if event.type == JOYHATMOTION:
                    self.on_hat_push(event.value[0], event.value[1])