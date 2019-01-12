#!/usr/bin/env python3
"""
Python remote control app for Ears prototype
"""

import sys
import time
import socket
import struct
import pygame

DRIVE_SCALE = 10.0
TURN_SCALE = 6.0

class RCCommand(struct.Struct):
    "Remote control command packet."

    COMMAND_MAGIC = b"apleseed"

    def __init__(self, ts=time.time(), fwd=0.0, turn=0.0, servos=[0.0]*4):
        "Create a command struct"
        super().__init__("8sddd4d")
        self.ts = ts
        self.fwd = fwd
        self.turn = turn
        self.servos = servos

    def pack(self):
        "Returns binary representation of command"
        return super().pack(self.COMMAND_MAGIC, self.ts, self.fwd, self.turn, *self.servos)

def run():
    """Container for remote "game" under pygame."""

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect(("192.168.43.0", 5555))
    screen_size = (120, 120)
    screen = pygame.display.set_mode(screen_size)
    screen.fill((0, 255, 0))
    running = True
    while running:
        try:
            event = pygame.event.poll()
            if event.type == pygame.QUIT:
                running = False
            elif event.type in (pygame.KEYDOWN, pygame.KEYUP):
                key = pygame.key.get_pressed()
                if key[pygame.K_q]: # q
                    running = False
                else:
                    cmd = RCCommand()
                    if key[pygame.K_UP]: # up arrow
                        cmd.fwd = DRIVE_SCALE
                    if key[pygame.K_DOWN]: # down arrow
                        cmd.fwd = -DRIVE_SCALE
                    if key[pygame.K_LEFT]: # left arrow
                        cmd.turn = TURN_SCALE
                    if key[pygame.K_RIGHT]: # right arrow
                        cmd.turn = -TURN_SCALE
                    if key[pygame.K_1]:
                        cmd.servos[0] = -1.5
                    if key[pygame.K_2]:
                        cmd.servos[0] = -1.0
                    if key[pygame.K_3]:
                        cmd.servos[0] = -0.5
                    if key[pygame.K_4]:
                        cmd.servos[0] = 0.0
                    if key[pygame.K_5]:
                        cmd.servos[0] = 0.5
                    if key[pygame.K_6]:
                        cmd.servos[0] = 1.0
                    if key[pygame.K_7]:
                        cmd.servos[0] = 1.5
                    sock.send(cmd.pack())
            time.sleep(0.05)
            pygame.display.flip()
        except Exception as exp:
            running = False
            sys.stderr.write(str(exp) + "\r\n")


if __name__ == '__main__':
    run()
