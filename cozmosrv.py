# -*- coding: utf-8 -*-
import io
import pycozmo
import logging
import socketserver
from threading import Condition
from threading import Timer
from threading import Lock
from PIL import ImageFont, ImageDraw, Image
import cv2
import traceback
import time
import json
import numpy as np
import os
import datetime as dt
from time import sleep
from sense_hat import SenseHat
from threading import Thread
from enum import Enum
JoystickModes = Enum("JoystickModes", "Lift Drive HeadTilt")

#import SimpleHTTPServer

from http.server import BaseHTTPRequestHandler,HTTPServer
sense = SenseHat()
sense.set_rotation(180)
pixels = []
pixelon = True
for i in range(0, 64):
    fract = i/64
    cf = int(255 * fract)
    pixels.append((cf, 255-cf, i))
sense.set_pixels(pixels)

robotstatusblinky = False
last_activity = time.time()
connected = False
connecting = False
one_shot_camera = True
one_shot_spf = 1
joystick_mode = JoystickModes.Drive
current_lift = 0.0
current_head_tilt = 0.0
address = ('', 4443)
framecount = 0
watchdog_robot_timeout = 20
disconnecting = False

PAGE="""\
<html>
<head>
<title>cozmoctrl</title>
<script language="javascript">

function up() {
	var oReq = new XMLHttpRequest();
	//oReq.addEventListener("load", reqListener);
	oReq.open("POST", "drive");
	oReq.send(JSON.stringify({ "lspeed": 100, "rspeed": 100, "duration" : 1 }));
}
function down() {
	var oReq = new XMLHttpRequest();
	oReq.open("POST", "drive");
	oReq.send(JSON.stringify({ "lspeed": -100, "rspeed": -100, "duration" : 1 }));
}
function left() {
	var oReq = new XMLHttpRequest();
	oReq.open("POST", "drive");
	oReq.send(JSON.stringify({ "lspeed": -100, "rspeed": 100, "duration" : 0.3 }));
}
function right() {
	var oReq = new XMLHttpRequest();
	oReq.open("POST", "drive");
	oReq.send(JSON.stringify({ "lspeed": 100, "rspeed": -100, "duration" : 0.3 }));
}
</script>
</head>
<body>
<h1>cozmo!</h1>
<img src="stream.mjpg" /><br/>
<span onClick="up();">⬆️</span>
<span onClick="down();">⬇️</span>
<span onClick="left();">⬅️</span>
<span onClick="right();">➡️</span>
</body>
</html>
"""

class StreamingOutput(object):
    def __init__(self):
        self.clientCount = 0
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()
        self.cozmoclient = None

    def retain(self):
        global connected
        print("retain")
        self.clientCount = self.clientCount + 1
        if self.clientCount == 1:
            camera_handler_setup()


    def release(self):
        self.clientCount = self.clientCount - 1
        print("release, %d clients remaining" % (self.clientCount))
        #if self.clientCount == 0:
            #print("no clients left, disabling camera")
            #self.cozmoclient.del_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image)
            
    def write(self, buf):
        #print("writing...")
        if buf.startswith(b'\x89\x50'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                #print("notifying waiters")
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(BaseHTTPRequestHandler):

    def do_POST(self):
        global connected
        global last_activity
        if not setupRobot():
            self.send_error(500, "No robot :-(")
            self.server.shutdown()
            return

        contentlength = self.headers['Content-Length']
        last_activity = time.time()
        if contentlength:
            c = int(contentlength)
            input = self.rfile.read(c)
            if (input == b''):
                self.send_response(401)
                self.end_headers()
                return
        else:
                self.send_response(402)
                self.end_headers()
                return        

        if self.path == '/image':
            print("loading image, got %d bytes" % (c))
            # Load image
            im = Image.open(io.BytesIO(input))
            # Convert to binary image.
            im = im.convert('1')
            self.server.cozmoclient.display_image(im)
            self.send_response(200)
            self.end_headers()
        elif self.path == '/tilt':
            range = pycozmo.MAX_HEAD_ANGLE.radians - pycozmo.MIN_HEAD_ANGLE.radians            
            angle = float(input) * range
            angle = angle + pycozmo.MIN_HEAD_ANGLE.radians
            print("setting head angle to %0.2f" % angle);
            self.server.cozmoclient.set_head_angle(angle)
            self.send_response(200)
            self.end_headers()
        elif self.path == '/headlight':
            enable = int(input)
            print("setting head ligt to %d" % enable);
            self.server.cozmoclient.set_head_light(enable)
            self.send_response(200)
            self.end_headers()            
        elif self.path == '/drive':
            data = json.loads(input)
            lspeed = int(data["lspeed"])
            rspeed = int(data["rspeed"])
            duration = float(data["duration"])
            print("*** drive: ", data)
            self.server.cozmoclient.drive_wheels(lwheel_speed=lspeed, rwheel_speed = rspeed, duration=duration)
            self.send_response(200)
            self.end_headers()
        elif self.path == '/turn':
            data = json.loads(input)
            speed = float(data["speed"])
            accel = float(data["accel"])
            direction = float(data["direction"])
            pkt = pycozmo.protocol_encoder.TurnInPlaceAtSpeed(wheel_speed_mmps=speed, wheel_accel_mmps2=accel, direction=direction)
            self.server.cozmoclient.conn.send(pkt)
            self.send_response(200)
            self.end_headers()
        elif self.path == '/lift':
            data = json.loads(input)
            current_lift = float(data["height"])
            height = (current_lift * (pycozmo.MAX_LIFT_HEIGHT.mm - pycozmo.MIN_LIFT_HEIGHT.mm)) + pycozmo.MIN_LIFT_HEIGHT.mm;
            #accel  = float(data["accel"])
            #max_speed = float(data["max_speed"])
            #duration  = float(data["duration"])
            self.server.cozmoclient.set_lift_height(height=height)
            self.send_response(200)
            self.end_headers()
                                          
        else:
            self.send_response(404)
            self.end_headers()
            
                
    def do_GET(self):
        global last_activity
        if not setupRobot():
            self.send_error(500, "No robot :-(")
            self.server.shutdown()
            return

        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/status':
            jsonstr = json.dumps(self.server.robotstatusdict)
            jsonbytes = jsonstr.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/json; charset=utf-8')
            self.send_header('Content-Length', len(jsonbytes))
            self.end_headers()
            self.wfile.write(jsonbytes)
            last_activity = time.time()
        elif self.path == '/stream.mjpg':
            output.retain()
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                        if (frame == b''):
                            print("skipping empty frame")
                            continue
                        
                        cv2.CV_LOAD_IMAGE_COLOR = 1 # set flag to 1 to give colour image

                        pil_im = Image.open(io.BytesIO(frame))

                        draw = ImageDraw.Draw(pil_im)

                        # Choose a font
                        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
                        myText = self.server.robotstatus + " ";
                        if (self.server.robotcharging == 1):
                            myText = myText +"(charging) "
                        elif (self.server.robotcharging == 0):
                            myText = myText + "(not charging) "
                        else:
                            myText = myText + "(?) "

                        myText = myText + dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                        # Draw the text
                        color = 'rgb(255,255,255)'
                        #draw.text((0, 0), myText,fill = color, font=font)

                        # get text size
                        text_size = font.getsize(myText)

                        # set button size + 10px margins
                        button_size = (text_size[0]+20, text_size[1]+10)

                        # create image with correct size and black background
                        button_img = Image.new('RGBA', button_size, "black")
                        
                        #button_img.putalpha(128)
                        # put text on button with 10px margins
                        button_draw = ImageDraw.Draw(button_img)
                        button_draw.text((10, 5), myText, fill = color, font=font)

                        # put button on source image in position (0, 0)

                        pil_im.paste(button_img, (0, 0))
                        bg_w, bg_h = pil_im.size 

                        # Save the image
                        buf= io.BytesIO()
                        pil_im.save(buf, format= 'JPEG')
                        frame = buf.getvalue()
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                    last_activity = time.time()

            except Exception as e:
                traceback.print_exc()
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
                output.release()
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def camera_handler_setup():
    connectLock.acquire(blocking=True)
    if server.cozmoclient:
        if not one_shot_camera:
            print("enabling camera")
        server.cozmoclient.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image, one_shot=one_shot_camera)
    else:
        print("no Cozmo, not adding camera handler")
    connectLock.release()

def watchdog():
    global disconnecting
    if hasattr(server.robotstatusdict, "client_timestamp"):
        last_status_report = server.robotstatusdict["client_timestamp"]
        time_since_last_report = time.time() - last_status_report
        if time_since_last_report > watchdog_robot_timeout:
            print("No robot status received in %d seconds, exiting")
            server.cozmoclient.disconnect()
            disconnecting = False
            quit()
    t = Timer(10, watchdog)
    t.start()

def on_camera_image(cli, image):
    del cli
    global framecount
    framecount = framecount + 1
    image.save(output, "png")

    server.robotstatusdict["last_image_acquisition"] = time.time()

    if one_shot_camera and output.clientCount > 0:
        t = Timer(one_shot_spf, camera_handler_setup)
        t.start()

def on_robot_charging(cli, state):
    del cli
    if state:
        print("Started charging.")
        server.robotcharging = 1
    else:
        print("Stopped charging.")
        server.robotcharging = 0        

def on_robot_state(cli, pkt: pycozmo.protocol_encoder.RobotState):
    #server.robotstatus = "B: {:.01f}V g x:{:.01f} y:{:.01f} z:{:.01f}".format(pkt.battery_voltage, pkt.gyro_x, pkt.gyro_y, pkt.gyro_z)
    server.robotstatus = "B: {:.01f}V".format(pkt.battery_voltage)
    newdict = { "battery_voltage" : pkt.battery_voltage,
                "robot_timestamp" : pkt.timestamp,
                "status_timestamp" : time.time(),
                "accel" : { "x" : pkt.accel_x,
                            "y" : pkt.accel_y,
                            "z" : pkt.accel_z },
                "gyro"  : { "x" : pkt.gyro_x,
                            "y" : pkt.gyro_y,
                            "z" : pkt.gyro_z },
                "backpack_touch_sensor_raw" : pkt.backpack_touch_sensor_raw,
                "pose" : { "x" : pkt.pose_x,
                           "y" : pkt.pose_y,
                           "z" : pkt.pose_z },
                "lift_height_mm" : pkt.lift_height_mm,
                "head_angle_rad" : pkt.head_angle_rad,
                "lift_height_max_mm" : pycozmo.MAX_LIFT_HEIGHT.mm,
                "lift_height_min_mm" : pycozmo.MIN_LIFT_HEIGHT.mm,
                "head_angle_min_rad" : pycozmo.MIN_HEAD_ANGLE.radians,
                "head_angle_max_rad" : pycozmo.MAX_HEAD_ANGLE.radians,
                "status" : pkt.status }
    if hasattr(server, 'robotstatusdict'):
        server.robotstatusdict.update(newdict);
    else:
        server.robotstatusdict = newdict

    pixels = []
    global robotstatusblinky
    global connected, disconnecting
    for i in range(0,64):
        if disconnecting:
            pixels.append((64,64,0))
        elif not connected:
            pixels.append((32,0,0))
        elif connecting:
            pixels.append((0, 64, 0))
        elif i < 16:
            if pkt.status & (1 << i):
                pixels.append((0,0,128))
            else:
                pixels.append((128,128,128))
        else:
            if i == 16 and output.clientCount > 0:
              pixels.append((128,0,0))
            elif i == 17:
                if robotstatusblinky == False:
                    pixels.append((0, 128, 0))
                    robotstatusblinky = True
                else:
                    pixels.append((0,0,0))
                    robotstatusblinky = False
            elif not disconnecting and (i >= 24 and i < 32):
                idletime = time.time() - last_activity
                #lastlight = (idletime/40) + 24
                lastlight = (idletime) + 24
                if lastlight > 32:
                    lastlight = 32
                if i <= lastlight:
                    pixels.append((int((i-24)*(128.0/8.0)), int((8-(i-24))*(128.0/8.0)), 0, ))
                else:
                    pixels.append((0,0,0))
                if lastlight == 32:
                    if not disconnecting:
                        print("idle disconnect")
                        server.cozmoclient.disconnect()
                        server.cozmoclient = None
                        disconnecting = True
                        connected = False
                    else:
                        print("pending disconnection")

            else:
                pixels.append((0,0,0))
    if not connected and not connecting:
        pixels = []
        for i in range(0,64):
            r = i
            g = 0
            b = 0
            pixels.append((r, g, b))

    sense.set_pixels(pixels)


def joystickthread():
    global last_activity, joystick_mode, current_lift, current_head_tilt
    while True:
        event = sense.stick.wait_for_event()
        last_activity = time.time()
        print("Joystick: {} {} | mode: {}".format(event.action, event.direction, joystick_mode))
        if event.action != "pressed":
            continue

        if not setupRobot():
            exit(2)

        if event.direction == "middle":
            modes = list(JoystickModes)
            new_mode = joystick_mode.value # value is index+1
            if new_mode+1 > len(modes):
                new_mode = 0

            joystick_mode = modes[new_mode]
            print("New joystick mode is {}".format(joystick_mode))

        if joystick_mode == JoystickModes.HeadTilt:
            if event.direction == "up":
                current_head_tilt = current_head_tilt + 0.1

            if event.direction == "down":
                current_head_tilt = current_head_tilt - 0.1

            if current_head_tilt > 1.0:
                current_head_tilt = 1.0
            if current_head_tilt < 0.0:
                current_head_tilt = 0

            range = pycozmo.MAX_HEAD_ANGLE.radians - pycozmo.MIN_HEAD_ANGLE.radians
            angle = float(current_head_tilt) * range
            angle = angle + pycozmo.MIN_HEAD_ANGLE.radians
            print("setting head angle to %0.2f" % angle)
            server.cozmoclient.set_head_angle(angle)

            
        if joystick_mode == JoystickModes.Drive:
            if event.direction == "up":
                server.cozmoclient.drive_wheels(lwheel_speed=50, rwheel_speed = 50, duration=0.3)
            if event.direction == "down":
                server.cozmoclient.drive_wheels(lwheel_speed=-50, rwheel_speed = -50, duration=0.3)
            if event.direction == "left":
                server.cozmoclient.drive_wheels(lwheel_speed=-50, rwheel_speed = 50, duration=0.3)
            if event.direction == "right":
                server.cozmoclient.drive_wheels(lwheel_speed=50, rwheel_speed = -50, duration=0.3)

        if joystick_mode == JoystickModes.Lift:
            if event.direction == "up":
                current_lift = current_lift + 0.1

            if event.direction == "down":
                current_lift = current_lift - 0.1

            if current_lift > 1.0:
                current_lift = 1.0
            if current_lift < 0.0:
                current_lift = 0

            height = (current_lift * (pycozmo.MAX_LIFT_HEIGHT.mm - pycozmo.MIN_LIFT_HEIGHT.mm)) + pycozmo.MIN_LIFT_HEIGHT.mm;
            server.cozmoclient.set_lift_height(height=height)

connectLock = Lock()

def setupRobot():
    global connecting, connected, disconnecting
    print("acquiring connect lock")
    connectLock.acquire(blocking=True)
    print("acquired connect lock")
    if connected:
        print("already connected")
        connectLock.release()
        return True

    connecting = True
    disconnecting = False

    try:
        cli = pycozmo.client.Client()
        cli.start()
        print("connecting")
        cli.connect()
        print("waiting for robot")
        cli.wait_for_robot()
        server.cozmoclient = cli
        print("robot connected")
        cli.add_handler(pycozmo.event.EvtRobotChargingChange, on_robot_charging)
        #angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
        cli.set_head_angle(0.1)
        server.robotcharging = -1
        
        cli.conn.add_handler(pycozmo.protocol_encoder.RobotState, on_robot_state, one_shot=False)
    
        pkt = pycozmo.protocol_encoder.EnableCamera()
        cli.conn.send(pkt)
        #pkt = pycozmo.protocol_encoder.EnableColorImages(enable=True)
        
        cli.conn.send(pkt)
        
        connected = True
        connecting = False

        connectLock.release()
        print("setup done.")
    
        return True
    except Exception as e:
        connectLock.release()
        print("failed to connect! " + str(e))
        connecting = False
        return False

def run():
    global connected
    if not setupRobot():
        exit(2)

    timer = Timer(10, watchdog)
    timer.setDaemon(True)
    timer.start()
    joysticker = Thread(target=joystickthread)
    joysticker.setDaemon(True)
    joysticker.start()
    try:
        server.serve_forever()
    finally:
        print("goodbye")


output = StreamingOutput()
server = StreamingServer(address, StreamingHandler)
run()
