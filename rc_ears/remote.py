#!/usr/bin/env python3
"""
Python remote control app for Ears prototype
"""

import time
import csv
import socket
import struct
import pygame

DRIVE_SCALE = 10.0
TURN_SCALE = 6.0
NUM_SERVOS = 4

# Disable warning about members defined outside init method, I don't know a cleaner way to do it and also have the
# update method below.
# pylint: disable=W0201
class RCCommand(struct.Struct):
    "Remote control command packet."

    COMMAND_MAGIC = b"apleseed"

    def __init__(self, timestamp=time.time(), fwd=0.0, turn=0.0, servos=(0.0,)*NUM_SERVOS):
        "Create a command struct"
        super().__init__("8sddd4d")
        self.update(fwd, turn, servos, timestamp)

    def update(self, fwd, turn, servos, timestamp=time.time()):
        "Update the contents of the command"
        self.timestamp = timestamp
        self.fwd = fwd
        self.turn = turn
        self.servos = list(servos)

    def pack(self):
        "Returns binary representation of command"
        return super().pack(self.COMMAND_MAGIC, self.timestamp, self.fwd, self.turn, *self.servos)


class Animation():
    "A container for a CSV file to use as an animation"

    FRAME_TIME = 1.0/30.0

    def __init__(self, csv_file, columns=("s0", "s1", "s2", "s4")):
        "Reads the contents of a CSV file inteprets as columns"
        reader = csv.reader(csv_file, delimiter=",")
        self.frames = []
        columns_indexes = {col: ind for col, ind in zip(columns)}
        for row in reader:
            timestamp = len(self.frames) * self.FRAME_TIME
            fwd = float(row[columns_indexes['fwd']]) if "fwd" in columns_indexes else 0.0
            turn = float(row[columns_indexes['turn']]) if "turn" in columns_indexes else 0.0
            servos = [0.0] * NUM_SERVOS
            for servo in range(NUM_SERVOS):
                col_name = f"s{servo}"
                if col_name in columns_indexes:
                    servos[servo] = float(row[columns_indexes[col_name]])
            self.frames.append(RCCommand(timestamp, fwd, turn, servos))

    def start(self):
        "Return a generator for frames"
        for frame in self.frames:
            yield frame

# Pygame's members are all runtime defined so don't warn about them
# pylint: disable=no-member
class Remote():
    "Class container for remote control"

    def __init__(self, host=("192.168.43.0", 5555)):
        "Sets up the remote with UDP connection to robot"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect(host)

    def __del__(self):
        "Make sure we send a stop command before tearing down the socket"
        self.send(RCCommand())

    def send(self, command):
        "Sends a command over the socket"
        self.sock.send(command.pack())

    def handle_key(self, key):
        "Handle pygame key events"
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
        return cmd

    def run(self):
        """Container for remote "game" under pygame."""
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
                        cmd = self.handle_key(key)
                        self.send(cmd)
                pygame.display.flip()
                time.sleep(Animation.FRAME_TIME)
            except KeyboardInterrupt:
                running = False

if __name__ == '__main__':
    Remote().run()
