#!/usr/bin/env python3
"""
Python remote control app for Ears prototype
"""

import time
import csv
import socket
import struct
import argparse
try:
    import pygame
except ImportError:
    ANS = input("Could not import pygame library, attempt to install automatically? [Y/n]: ")
    if not ANS or ANS.lower().startswith('y'):
        from subprocess import call
        if call(['pip3', 'install', 'pygame']) == 0:
            import pygame
        else:
            exit("Could not automatically install pygame. You may need to try installing it manually with sudo.\n" \
                 "e.g.:\tsudo pip3 install pygame")
    else:
        exit("Could not import pygame library, it needs to be installed to run this program.")



DRIVE_SCALE = 16.0
ANIM_DRIVE_SCALE = 100.0
TURN_SCALE = 6.0
ANIM_TURN_SCALE = 1.0
SERVO_SCALES = (-1.25/90.0, 1.25/90.0, 0.0, 0.0)
SERVO_OFFSETS = (0.0, 0.0, 0.0, 0.0)
NUM_SERVOS = 4
FRAME_TIME = 1.0/30.0

# Disable warning about members defined outside init method, I don't know a cleaner way to do it and also have the
# update method below.
# pylint: disable=W0201
# pylint: disable=E1121
# Disable review flags
# pylint: disable=R

class Button:
    "A class for rendering pygame buttons"

    def __init__(self, color, origin_x, origin_y, length, height, width, text, text_color):
        "Initalize the button instance"
        self.color = color
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.length = length
        self.height = height
        self.width = width
        self.text = text
        self.text_color = text_color
        self.rect = pygame.Rect(origin_x, origin_y, length, height)

    def _write_text(self, surface, text_color):
        font_size = int(self.length//len(self.text))
        my_font = pygame.font.SysFont("Calibri", font_size)
        my_text = my_font.render(self.text, 1, text_color)
        surface.blit(my_text, ((self.origin_x+self.length/2) - my_text.get_width()/2,
                               (self.origin_y+self.height/2) - my_text.get_height()/2))
        return surface

    def _draw_button(self, surface, color):
        "Render the button to serface"
        for i in range(1, 10):
            surface_i = pygame.Surface((self.length+(i*2), self.height+(i*2)))
            surface_i.fill(color)
            alpha = (255/(i+2))
            if alpha <= 0:
                alpha = 1
            surface_i.set_alpha(alpha)
            pygame.draw.rect(surface_i, color,
                             (self.origin_x-i, self.origin_y-i, self.length+i, self.height+i), self.width)
            surface.blit(surface_i, (self.origin_x-i, self.origin_y-i))
        pygame.draw.rect(surface, color, (self.origin_x, self.origin_y, self.length, self.height), 0)
        pygame.draw.rect(surface, (190, 190, 190), (self.origin_x, self.origin_y, self.length, self.height), 1)
        return surface

    def update_button(self, surface, color=None, text_color=None):
        "Function to draw to rerender button each frame"
        if color is None:
            color = self.color
        if text_color is None:
            text_color = self.text_color
        surface = self._draw_button(surface, color)
        surface = self._write_text(surface, text_color)
        return surface

    def pressed(self, mouse):
        "Check if a mouse click event was inside the button"
        if mouse[0] > self.rect.topleft[0]:
            if mouse[1] > self.rect.topleft[1]:
                if mouse[0] < self.rect.bottomright[0]:
                    if mouse[1] < self.rect.bottomright[1]:
                        return True
        return False


class RCCommand(struct.Struct):
    "Remote control command packet."

    COMMAND_MAGIC = b"apleseed"

    def __init__(self, timestamp=time.time(), fwd=0.0, turn=0.0, servos=(0.0,)*NUM_SERVOS, flags=0x0):
        "Create a command struct"
        super().__init__("8sddd4d4xI")
        self.update(fwd, turn, servos, timestamp, flags)

    def __repr__(self):
        "Debuggable represenation of command"
        return "RCCommand({0.timestamp:0.3f}, {0.fwd:0.3f}, {0.turn:0.3f}, {0.servos!r})".format(self)

    def update(self, fwd, turn, servos, timestamp=time.time(), flags=0x0):
        "Update the contents of the command"
        self.timestamp = timestamp
        self.fwd = fwd
        self.turn = turn
        self.servos = list(servos)
        self.flags = flags

    def copy(self):
        "Return a new RCCommand which is a copy of this one"
        return RCCommand(self.timestamp, self.fwd, self.turn, self.servos, self.flags)

    def pack(self):
        "Returns binary representation of command"
        return super().pack(self.COMMAND_MAGIC, self.timestamp, self.fwd, self.turn, *self.servos, self.flags)


def load_animation(csv_file, columns=("s0", "s1", "s2", "s4"), skip=0, trim=None, mirror=False, append_recenter=False):
    "Reads the contents of a CSV file inteprets according to columns into an animation frame list"
    reader = csv.reader(csv_file, delimiter="\t")
    frames = []
    columns_indexes = {col: ind for ind, col in enumerate(columns)}
    if mirror and 's0' in columns_indexes and 's1' in columns_indexes:
        columns_indexes['s0'], columns_indexes['s1'] = columns_indexes['s1'], columns_indexes['s0']
    def get_cmds(row):
        "internal function to get the cmds"
        if "fwd" in columns_indexes:
            pos = float(row[columns_indexes['fwd']])
            fwd = (pos - get_cmds.prev_pos) * ANIM_DRIVE_SCALE
            get_cmds.prev_pos = pos
        else:
            fwd = 0.0
        if "turn" in columns_indexes:
            heading = float(row[columns_indexes['turn']])
            turn = (heading - get_cmds.prev_heading) * ANIM_TURN_SCALE
            if mirror:
                turn *= -1.0
            get_cmds.prev_heading = heading
        else:
            turn = 0.0
        servos = [0.0] * NUM_SERVOS
        for servo in range(NUM_SERVOS):
            col_name = f"s{servo}"
            if col_name in columns_indexes:
                servos[servo] = float(row[columns_indexes[col_name]]) * SERVO_SCALES[servo] + SERVO_OFFSETS[servo]
        return fwd, turn, servos
    get_cmds.prev_pos = 0.0
    get_cmds.prev_heading = 0.0
    for row in reader:
        if skip:
            skip -= 1
            get_cmds(row) # Skill need to process row to keep prev_pos and prev_heading correct
            continue
        if trim and len(frames) >= trim:
            break
        timestamp = len(frames) * FRAME_TIME
        fwd, turn, servos = get_cmds(row)
        frames.append(RCCommand(timestamp, fwd, turn, servos))
    if append_recenter:
        print("ARC:", len(frames))
        # Reread the file with the turn reversed and servos centered
        csv_file.seek(0)
        for row in reader:
            timestamp = len(frames) * FRAME_TIME
            _, turn, _ = get_cmds(row)
            if not mirror:
                turn *= -1
            frames.append(RCCommand(timestamp, 0.0, turn, SERVO_OFFSETS))
        print(len(frames))
    return frames

TURN_BREAK_IND = 14

# Pygame's members are all runtime defined so don't warn about them
# pylint: disable=no-member
class Remote():
    "Class container for remote control"

    def __init__(self, host, animated_drive=False):
        "Sets up the remote with UDP connection to robot"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.animated_drive = animated_drive
        self._drive_target = 0.0
        self._turn_target = 0.0
        self.animation = []
        self.animations = {
            "r45": load_animation(open('45deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"), mirror=False),
            "l45": load_animation(open('45deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"), mirror=True),
            "forward_drive": load_animation(open('fwd_drive_01.csv'), ("s1", "s0", "fwd")),
            "forward_fast": load_animation(open('fwd_drive_05.csv'), ("s1", "s0", "fwd")),
            "backward_drive": load_animation(open('drvback_01.csv'), ("s1", "s0", "fwd")),
            "right": load_animation(open('90deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"),
                                    mirror=False, append_recenter=False),
            "left": load_animation(open('90deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"),
                                   mirror=True, append_recenter=False),
            "wiggle1": load_animation(open('earWiggle_01.csv'), ("s1", "s0", "fwd", "turn")),
            "wiggle2": load_animation(open('earWiggle_02.csv'), ("s1", "s0", "fwd", "turn")),
            "hurt1": load_animation(open('hurt_01.csv'), ("s1", "s0", "fwd", "turn")),
            "proc_fwd_enter": load_animation(open('fwd_drive_05.csv'), ("s1", "s0", "fwd"), skip=0, trim=17),
            "proc_fwd_loop": load_animation(open('fwd_drive_05.csv'), ("s1", "s0", "fwd"), skip=17, trim=55-17),
            "proc_fwd_exit": load_animation(open('fwd_drive_05.csv'), ("s1", "s0", "fwd"), skip=55, trim=None),
            "proc_back_enter": load_animation(open('drvback_01.csv'), ("s1", "s0", "fwd"), skip=0, trim=19),
            "proc_back_loop": load_animation(open('drvback_01.csv'), ("s1", "s0", "fwd"), skip=19, trim=42-19),
            "proc_back_exit": load_animation(open('drvback_01.csv'), ("s1", "s0", "fwd"), skip=42, trim=None),
            "proc_right_enter": load_animation(open('45deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"),
                                               trim=TURN_BREAK_IND, mirror=False),
            "proc_right_exit": load_animation(open('45deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"),
                                              skip=TURN_BREAK_IND, mirror=False),
            "proc_left_enter": load_animation(open('45deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"),
                                              trim=TURN_BREAK_IND, mirror=True),
            "proc_left_exit": load_animation(open('45deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"),
                                             skip=TURN_BREAK_IND, mirror=True),
            #"right": load_animation(open('45deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"),
            #                        mirror=False, append_recenter=True),
            #"left": load_animation(open('45deg_turn_right_01.csv'), ("s1", "s0", "fwd", "turn"),
            #                       mirror=True, append_recenter=True),
        }
        for cmd in self.animations['proc_fwd_loop']: # Override speed target for forward cruise animation
            cmd.fwd = DRIVE_SCALE
        for cmd in self.animations['proc_back_loop']: # Override speed target for forward cruise animation
            cmd.fwd = -DRIVE_SCALE
        self.proc_right_pos = self.animations["proc_right_exit"][0].servos
        self.proc_left_pos = self.animations["proc_left_exit"][0].servos
        self.turn_script = []
        self.turn_script_last_dir = None
        self.sock.connect(host)


    def __del__(self):
        "Make sure we send a stop command before tearing down the socket"
        self.send(RCCommand())

    def send(self, command):
        "Sends a command over the socket"
        self.sock.send(command.pack())

    @property
    def drive_target(self):
        "Driving target speed."
        return self._drive_target
    @drive_target.setter
    def drive_target(self, speed):
        if self.animated_drive:
            if speed > 0.0:
                if self._drive_target < speed:
                    self.play_anim("proc_fwd_enter", False)
            elif speed < 0.0:
                if self._drive_target > speed:
                    self.play_anim("proc_back_enter", False)
            else: # Speed == 0
                if self._drive_target > 0.0:
                    self.play_anim("proc_fwd_exit", False)
                elif self._drive_target < 0.0:
                    self.play_anim("proc_back_exit", "False")
        self._drive_target = speed

    @property
    def turn_target(self):
        "Turn target rate"
        return self._turn_target
    @turn_target.setter
    def turn_target(self, rate):
        if self.animated_drive:
            if self.drive_target == 0.0:
                if rate > 0.0:
                    if self._turn_target < rate:
                        self.play_anim("proc_left_enter", False)
                elif rate < 0.0:
                    if self._turn_target > rate:
                        self.play_anim("proc_right_enter", False)
                else: # rate == 0
                    if self._turn_target > 0.0:
                        self.play_anim("proc_left_exit", False)
                    elif self._turn_target < 0.0:
                        self.play_anim("proc_right_exit", False)
        self._turn_target = rate

    def play_anim(self, name, clear_targets=True):
        "Queue playing a given animation"
        print("Animating", name)
        if clear_targets:
            self._drive_target = 0
            self._turn_target = 0
        self.animation = self.animations[name].copy()

    def start_turn_script(self, list_name):
        "Starts running a turn in place animation script"
        self.send(RCCommand(flags=0x01))
        self.turn_script_last_dir = None
        self.turn_script = [turn.strip() for turn in open(list_name)]

    def stop_all(self):
        "Stop all currently playing animations, scripts, etc."
        self.turn_script = []
        self.animation = []
        self._drive_target = 0.0
        self._turn_target = 0.0

    def handle_key(self, event):
        "Handle pygame key events"
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP: # up arrow
                self.drive_target = DRIVE_SCALE
            if event.key == pygame.K_DOWN: # down arrow
                self.drive_target = -DRIVE_SCALE
            if event.key == pygame.K_LEFT: # left arrow
                self.turn_target = TURN_SCALE
            if event.key == pygame.K_RIGHT: # right arrow
                self.turn_target = -TURN_SCALE
            if event.key == pygame.K_l:
                self.play_anim("l45")
            if event.key == pygame.K_r:
                self.play_anim("r45")
            if event.key == pygame.K_k:
                self.play_anim("left")
            if event.key == pygame.K_t:
                self.play_anim("right")
            if event.key == pygame.K_f:
                self.play_anim("forward_drive")
            if event.key == pygame.K_g:
                self.play_anim("forward_fast")
            if event.key == pygame.K_b:
                self.play_anim("backward_drive")
            if event.key == pygame.K_w:
                self.play_anim("wiggle1")
            if event.key == pygame.K_s:
                self.play_anim("wiggle2")
            if event.key == pygame.K_h:
                self.play_anim("hurt1")
            if event.key == pygame.K_1:
                self.start_turn_script("random_turns1.csv")
            if event.key == pygame.K_2:
                self.start_turn_script("random_turns2.csv")
            if event.key == pygame.K_3:
                self.start_turn_script("random_turns3.csv")
            if event.key == pygame.K_0:
                self.start_turn_script("random_turns0.csv")
            if event.key == pygame.K_SPACE:
                self.stop_all()
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                self.drive_target = 0.0
            if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                self.turn_target = 0.0

    def _make_buttons(self):
        "Return a bunch of buttons to play different animations"
        buttons = (("Wiggle", lambda: self.play_anim("wiggle1")),
                   ("Excited", lambda: self.play_anim("wiggle2")),
                   ("Hurt", lambda: self.play_anim("hurt1")),
                   ("Stop all", self.stop_all)
                  )
        size = 200
        widgets = []
        for title, anim in buttons:
            widgets.append((Button((100, 100, 100), 10, 10 + (size//2 + 10)*len(widgets), size, size//2, 0,
                                   title, (255, 255, 255)), anim))
        return widgets


    def run(self):
        """Container for remote "game" under pygame."""
        pygame.init()
        screen_size = (220, 480)
        screen = pygame.display.set_mode(screen_size)
        buttons = self._make_buttons()
        running = True
        while running:
            screen.fill((0, 128, 255))
            for button, _ in buttons:
                button.update_button(screen)
            try:
                event = pygame.event.poll()
                if event.type == pygame.QUIT:
                    running = False
                elif event.type in (pygame.KEYDOWN, pygame.KEYUP):
                    if event.key == pygame.K_q: # q
                        running = False
                    else:
                        self.handle_key(event)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mouse_pos = pygame.mouse.get_pos()
                    for button, action in buttons:
                        if button.pressed(mouse_pos):
                            action()
                if self.animation:
                    cmd = self.animation.pop(0).copy()
                    if self.animated_drive and self.drive_target != 0.0: # When animated driving forward or back
                        if self.turn_target > 0.0:
                            cmd.turn = self.turn_target
                            cmd.servos = self.proc_left_pos
                        elif self.turn_target < 0.0:
                            cmd.turn = self.turn_target
                            cmd.servos = self.proc_right_pos
                    self.send(cmd)
                elif self.turn_script:
                    #self.send(RCCommand(flags=0x02))
                    if self.turn_script_last_dir:
                        for _ in range(15):
                            self.send(RCCommand(turn=self.turn_script_last_dir * -1.0))
                            time.sleep(FRAME_TIME)
                    for _ in range(30):
                        self.send(RCCommand())
                        time.sleep(FRAME_TIME)
                    anim = self.turn_script.pop(0)
                    print("Script remaining:", len(self.turn_script))
                    self.turn_script_last_dir = TURN_SCALE if anim == "left" else -TURN_SCALE
                    self.play_anim(anim)
                elif self.animated_drive and (self.drive_target != 0.0 or self.turn_target != 0.0):
                    if self.drive_target > 0.0:
                        self.play_anim("proc_fwd_loop", False)
                    elif self.drive_target < 0.0:
                        self.play_anim("proc_back_loop", False)
                    else:
                        self.send(RCCommand(fwd=self.drive_target,
                                            turn=self.turn_target,
                                            servos=self.proc_left_pos if self.turn_target > 0.0 else
                                            self.proc_right_pos))
                else:
                    self.send(RCCommand(fwd=self.drive_target, turn=self.turn_target, servos=SERVO_OFFSETS))
                pygame.display.flip()
                time.sleep(FRAME_TIME)
            except KeyboardInterrupt:
                running = False


def main():
    "Program entry point"
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--ip_address', default="192.168.8.1", help="Specify robot's ip address")
    parser.add_argument('-p', '--port', default=5555, type=int, help="Manually specify robot's port")
    args = parser.parse_args()
    Remote((args.ip_address, args.port), True).run()

if __name__ == '__main__':
    main()
