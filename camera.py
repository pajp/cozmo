#!/usr/bin/env python

from PIL import Image

import time

import pycozmo


def on_robot_state(cli, pkt: pycozmo.protocol_encoder.RobotState):
    del cli
    print("Battery level: {:.01f} V".format(pkt.battery_voltage))


def on_robot_poked(cli, pkt: pycozmo.protocol_encoder.RobotPoked):
    del cli, pkt
    print("Robot poked.")


def on_robot_falling_started(cli, pkt: pycozmo.protocol_encoder.FallingStarted):
    del cli, pkt
    print("Started falling.")


def on_robot_falling_stopped(cli, pkt: pycozmo.protocol_encoder.FallingStopped):
    del cli
    print("Falling stopped after {} ms. Impact intensity {:.01f}.".format(pkt.duration_ms, pkt.impact_intensity))


def on_button_pressed(cli, pkt: pycozmo.protocol_encoder.ButtonPressed):
    del cli
    if pkt.pressed:
        print("Button pressed.")
    else:
        print("Button released.")


def on_robot_picked_up(cli, state):
    del cli
    if state:
        print("Picked up.")
    else:
        print("Put down.")


def on_robot_charging(cli, state):
    del cli
    if state:
        print("Started charging.")
    else:
        print("Stopped charging.")


def on_cliff_detected(cli, state):
    del cli
    if state:
        print("Cliff detected.")


def on_robot_wheels_moving(cli, state):
    del cli
    if state:
        print("Started moving.")
    else:
        print("Stopped moving.")


def on_camera_image(cli, image):
    del cli
    image.save(f'capture/camera-{time.time()}.png', "PNG")


def pycozmo_program(cli: pycozmo.client.Client):

    cli.conn.add_handler(pycozmo.protocol_encoder.RobotState, on_robot_state, one_shot=True)
    cli.conn.add_handler(pycozmo.protocol_encoder.RobotPoked, on_robot_poked)
    cli.conn.add_handler(pycozmo.protocol_encoder.FallingStarted, on_robot_falling_started)
    cli.conn.add_handler(pycozmo.protocol_encoder.FallingStopped, on_robot_falling_stopped)
    cli.conn.add_handler(pycozmo.protocol_encoder.ButtonPressed, on_button_pressed)
    cli.add_handler(pycozmo.event.EvtRobotPickedUpChange, on_robot_picked_up)
    cli.add_handler(pycozmo.event.EvtRobotChargingChange, on_robot_charging)
    cli.add_handler(pycozmo.event.EvtCliffDetectedChange, on_cliff_detected)
    cli.add_handler(pycozmo.event.EvtRobotWheelsMovingChange, on_robot_wheels_moving)


    heart = Image.open("hjarta.png")
    heart = heart.convert('1')
    eyes = Image.open("eyes.png")
    eyes = eyes.convert('1')

    cli.display_image(eyes)
    
    angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
    #cli.set_head_angle(angle)
    cli.set_head_angle(0)

    pkt = pycozmo.protocol_encoder.EnableCamera(enable=True)
    cli.conn.send(pkt)
    pkt = pycozmo.protocol_encoder.EnableColorImages(enable=True)
    cli.conn.send(pkt)

    # Wait for image to stabilize.
    time.sleep(1.0)

    cli.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image, one_shot=False)
    speed = 100
    cli.drive_wheels(lwheel_speed=-speed,rwheel_speed=-speed, duration=0.5)
    cli.drive_wheels(lwheel_speed=speed,rwheel_speed=speed, duration=1.0)
    cli.drive_wheels(lwheel_speed=speed,rwheel_speed=speed, duration=3.0)
    time.sleep(1)
    cli.display_image(heart)
    time.sleep(1)
    cli.display_image(eyes)
    
    turnduration=1.38
    cli.drive_wheels(lwheel_speed=speed,rwheel_speed=-speed, duration=turnduration)
    cli.drive_wheels(lwheel_speed=speed,rwheel_speed=speed, duration=2.25)
    time.sleep(1)
    cli.drive_wheels(lwheel_speed=speed,rwheel_speed=-speed, duration=turnduration)
    cli.display_image(heart)
    cli.drive_wheels(lwheel_speed=-speed,rwheel_speed=-speed, duration=1.7)
    cli.display_image(eyes)    
    time.sleep(1)


pycozmo.run_program(pycozmo_program)
