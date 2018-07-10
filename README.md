# GenCyber 2018

Authors: Kali Regenold, Remington Bullis

This repository will hold the development code for the GenCyber Camp Robotics Class.
Everyday of the camp, the stable code here will be duplicated into a daily folder in the GenCyber18 repo, aka the production repo.

```python
import geekbot
from time import sleep

inigo_montoya = geekbot.Robot(57600)
if not inigo_montoya.connected:
    exit()

# Check lights
inigo_montoya.lights_on()
sleep(.5)
inigo_montoya.lights_off()

# Check IR servo and sensor
inigo_montoya.set_ir_position(0)
sleep(.25)
print(inigo_montoya.get_ir_distance())
sleep(.25)
inigo_montoya.set_ir_position(90)
sleep(.25)
print(inigo_montoya.get_ir_distance())
sleep(.25)
inigo_montoya.set_ir_position(-90)
sleep(.25)
print(inigo_montoya.get_ir_distance())
sleep(.25)
inigo_montoya.set_ir_position(0)
sleep(.5)

# Check left wheel servo
inigo_montoya.drive_left_wheel(90, seconds=1)
inigo_montoya.drive_left_wheel(-90, seconds=1)
inigo_montoya.drive_left_wheel(0, seconds=0.5)

# Check right wheel servo
inigo_montoya.drive_right_wheel(90, seconds=1)
inigo_montoya.drive_right_wheel(-90, seconds=1)
inigo_montoya.drive_right_wheel(0, seconds=0.5)

# Check drive forward and halt
inigo_montoya.drive_forward(80, seconds=1)

inigo_montoya.shutdown()
```
