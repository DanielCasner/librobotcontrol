# B24 Ears Remote

## Setup the robot
Power on the robot by pushing the topmost button under the face. It takes about 30 seconds to boot. During boot many LEDs will blink. When the robot is ready to run, the green LED next to the buttons should glow steadily and the ears will move slightly.

If you hold the robot upright, it will start attempting to balance.

If the robot tips over, it will stop balancing until held upright steadily again for a few seconds.

If you need to stop balancing without shutting down the robot, push the lower most button under the face to put it in pause mode, the green LED is replaced with a red one. Function can be resumed by pressing the pause button again.

To shutdown the robot, press the top most button under the face.

## Connect to robot WiFi Access point.
Connect your laptop to the WiFi access point broadcast by the robot.
- SSID: `BeagleBone-????`
- Password: `BeagleBone`

## Use the remote control
Open a terminal in the remote directory and run
```
python3 remote.py
```
This will open a blue window with some buttons.

Click the buttons to play specific animations.

With the python window in focused, the robot can be driven around using the arrow keys.
