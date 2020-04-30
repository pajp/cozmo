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
import random
from time import sleep
from sense_hat import SenseHat
from threading import Thread
from enum import Enum
import requests
from http.server import BaseHTTPRequestHandler,HTTPServer
import os.path as path

JoystickModes = Enum("JoystickModes", "Drive Speed Lift HeadTilt SpheroSelect SpheroDrive")

sense = SenseHat()
sense.set_rotation(0)
pixels = []
pixelon = True

black = (0,0,0)
red = (64,0,0)
green = (0,255,0)
blue = (0,0,255)
_X = red
_O = black
exitscreen = [
    _X, _O, _O, _O, _O, _O, _O, _X,
    _O, _X, _O, _O, _O, _O, _X, _O,
    _O, _O, _X, _O, _O, _X, _O, _O,
    _O, _O, _O, _X, _X, _O, _O, _O,
    _O, _O, _O, _X, _X, _O, _O, _O,
    _O, _O, _X, _O, _O, _X, _O, _O,
    _O, _X, _O, _O, _O, _O, _X, _O,
    _X, _O, _O, _O, _O, _O, _O, _X ];    
    
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
joystick_speed = 3
current_lift = 0.0
current_head_tilt = 0.0
address = ('', 4443)
framecount = 0
watchdog_robot_timeout = 20
timeout = 120.0
#timeout = 10
idle = 0
disconnecting = False
sphero_server = "http://The-Otter.local:4444/"
sphero_index = 0
sphero_uuid = None
sphero_attached = None
sphero_attaching = False
sphero_heading = 0
sphero_devices = []
sphero_scanning = False
sphero_idlescan_interval = 3600

cozmoblinky = False
led_brightness = 1.0

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
            self.send_error(503, "No robot :-(", "Unable to connect to Cozmo. I'll try to wake Cozmo up, please try again in a minute.")
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
        last_activity = time.time()
        if not setupRobot():
            self.send_error(503, "No robot :-(", "Unable to connect to Cozmo. I'll try to wake Cozmo up, please try again in a minute.")
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
            if hasattr(self.server, 'robotstatusdict'):
                jsonstr = json.dumps(self.server.robotstatusdict)
                jsonbytes = jsonstr.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/json; charset=utf-8')
                self.send_header('Content-Length', len(jsonbytes))
                self.end_headers()
                self.wfile.write(jsonbytes)
            else:
                self.send_error(404)
                self.end_headers()

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

def user_idle_watchdog():
    global idle, connected, disconnecting, last_activity
    global sphero_devices, sphero_attached, sphero_scanning
    print("user idle watchdog running")
    while True:
        idle = time.time() - last_activity
        if connected and idle >= timeout and not disconnecting:
            print("idle disconnect")
            disconnecting = True
            server.cozmoclient.disconnect()
            server.cozmoclient = None
            connected = False

        sleep(1)

def watchdog():
    global disconnecting
    if hasattr(server, "robotstatusdict") and hasattr(server.robotstatusdict, "client_timestamp"):
        last_status_report = server.robotstatusdict["client_timestamp"]
        time_since_last_report = time.time() - last_status_report
        if time_since_last_report > watchdog_robot_timeout:
            print("No robot status received in %d seconds, exiting")
            server.cozmoclient.disconnect()
            disconnecting = False
            quit()
        else:
            print("No robot status dict on server object - robot hasn't connected since startup")
            
    t = Timer(10, watchdog)
    t.start()

def on_camera_image(cli, image):
    del cli
    global framecount
    framecount = framecount + 1
    image.save(output, "png")

    if hasattr(server, "robotstatusdict"):
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

def update_led():
    global connected, disconnecting, idle
    global cozmoblinky
    global sphero_devices, sphero_index, sphero_blinky, sphero_uuid
    global led_brightness
    global pixels
    old_pixels = pixels.copy()
    pixels = []    
    for i in range(0,64):
        pixels.append((0,0,0))

    if idle > timeout:
        led_brightness = 0.4
    else:
        led_brightness = 1.0

    # first two rows = status bits from Cozmo status packet
    if hasattr(server, 'robotstatusdict') and connected:
        cozmostatus = server.robotstatusdict['status']
        for i in range(0,16): # 16 status bits
            if cozmostatus & (1 << i):
                pixels[i] = (0,0,126)
            else:
                pixels[i] = (128,128,128)

    for i in range(16,24): # 3rd row = number of streaming clients
        if i-16 >= output.clientCount:
            pixels[i] = (0,0,0)
        else:
            pixels[i] = (128,0,0)

    for i in range(24,32): # 4th row = joystick mode indicator
        if i-23 == joystick_mode.value:
            pixels[i] = (0,128,0)
        else:
            pixels[i] = (0,0,0)

    for i in range(32,40): # 5th row = speed indicator
        if i-31 <= joystick_speed:
            pixels[i] = (128,128,0)
        else:
            pixels[i] = (0,0,0)

    # 6th row = idle timeout indicator
    idle_fraction = idle / timeout
    idle_leds = 8 * idle_fraction

    for i in range(40,48):
        if idle < timeout:
            x = i-40
            if x <= idle_leds:
                c = int(255 * (x/8))
                pixels[i] = (c, 255-c, 0)
            else:
                pixels[i] = (0,0,0)
        else:
            pixels[i] = (128,0,0)

    # 7th row = sphero device indicator
    sd = sphero_devices.copy()
    if sphero_scanning:
        for i in range(0,8):
            r = random.randrange(255)
            g = random.randrange(255)
            b = random.randrange(255)
            pixels[48+i] = (r, g, b)
    else:
        for i in range(0,len(sd)):
            pxl = 48+i
            #print("comparing uuid {} with selected uuid {}".format(sphero_devices[i]["uuid"], sphero_uuid))
            
            if sphero_blinky and sphero_devices[i]["uuid"] == sphero_uuid:
                sphero_index = i
                pixels[pxl] = (255, 255, 255)
            elif sd[i]["name"].startswith("Lightning"):
                pixels[pxl] = (255, 0, 0)
            elif sd[i]["name"].startswith("R2"):
                pixels[pxl] = (0, 0, 255)
            elif sd[i]["name"].startswith("BB"):
                pixels[pxl] = (255, 255, 0)
            else:
                print("Unknown Sphero bot at index {}".format(i))

    if sphero_attached != None:
        pixels[62] = (0,128,0)
    elif sphero_attaching:
        pixels[62] = (128,128,0)
    else:
        pixels[62] = (128,0,0)

    cozmopxl = (0,0,0)
    if connecting:
        cozmopxl = (255,255,0)
    elif not connected:
        cozmopxl = (255,0,0)
    else:
        if not cozmoblinky:
            cozmopxl = (0,128,0)
        else:
            cozmopxl = (0,0,128)
    pixels[63] = cozmopxl

    for i in range(0,len(pixels)):
        r = int(float(pixels[i][0]) * led_brightness)
        g = int(float(pixels[i][1]) * led_brightness)
        b = int(float(pixels[i][2]) * led_brightness)
        pixels[i] = (r, g, b)

    if old_pixels != pixels:
        sense.set_pixels(pixels)

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

    global cozmoblinky
    cozmoblinky = not cozmoblinky

def attachToSphero(device):
    global sphero_attached, sphero_attaching
    sphero_attaching = True
    uuid = device["uuid"]
    print("attaching device: {}…".format(device))
    data = json.dumps({"uuid": uuid, "disconnectOthers" : True})
    req = requests.post(sphero_server + "attach", data = data);
    if req.status_code == 200:
        sphero_attached = uuid
        sphero_attaching = False
        print("ok")
    else:
        sphero_attached = None
        sphero_attaching = False
        print("failed")
    

last_held = 0.0
hold_count = 0
def joystickthread():
    global last_activity, joystick_mode, current_lift, current_head_tilt, joystick_speed, server
    global sphero_server, sphero_attached, sphero_attaching, sphero_heading, sphero_speed, sphero_index, sphero_uuid
    global last_held, hold_count
    global connected
    while True:
        event = sense.stick.wait_for_event()
        last_activity = time.time()
        print("Joystick: {} {} | mode: {}".format(event.action, event.direction, joystick_mode))
        if event.action == "released":
            if event.direction != "middle" and joystick_mode == JoystickModes.SpheroDrive:
                print("stopping Sphero")
                
                requests.post(sphero_server + "stop")
                continue
            else:
                continue

        min_time_between_holds = 0.4
        if joystick_mode == JoystickModes.SpheroDrive:
            throttle_limit = 2

        if event.action == "held":
            time_since_last_held = time.time() - last_held
            print("hold throttle: time since last hold: {}, hould_count: {}".format(time_since_last_held, hold_count))
            
            if time_since_last_held < min_time_between_holds:
                print("ignoring hold {}s since last hold ({})".format(time_since_last_held, last_held))
                continue
            last_held = time.time()


        if joystick_mode == JoystickModes.Drive:
            setupRobotInBackground()

        if event.direction == "middle":
            modes = list(JoystickModes)
            new_mode = joystick_mode.value # value is index+1
            if new_mode+1 > len(modes):
                new_mode = 0

            joystick_mode = modes[new_mode]
            print("New joystick mode is {}".format(joystick_mode))

            if joystick_mode == JoystickModes.SpheroSelect:
                backgroundSpheroScan()
                continue

        if joystick_mode == JoystickModes.SpheroSelect:
            if event.direction == "right":
                sphero_index = sphero_index + 1
                if sphero_index > len(sphero_devices)-1:
                    sphero_index = 0

            if event.direction == "left":
                sphero_index = sphero_index - 1
                if sphero_index < 0:
                    sphero_index = len(sphero_devices)-1

            sphero_uuid = sphero_devices[sphero_index]["uuid"]

            if len(sphero_devices) > 0:
                print("sphero index: {}, device: {}".format(sphero_index, sphero_devices[sphero_index]))
            else:
                print("sphero index: {}, no devices".format(sphero_index))
            

        if connected and joystick_mode == JoystickModes.HeadTilt:
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

        if joystick_mode == JoystickModes.Speed:
            if event.direction == "up":
                joystick_speed = joystick_speed + 1
                if joystick_speed > 8:
                    joystick_speed = 8
            if event.direction == "down":
                joystick_speed = joystick_speed - 1
                if joystick_speed < 1:
                    joystick_speed = 1

            if joystick_speed > 5:
                sphero_speed = 1
            else:
                sphero_speed = 2
                    
            print("new speed: %d" % joystick_speed)
        
        if connected and joystick_mode == JoystickModes.Drive:

            # picked speed increments 1/8 = 0,125 = 1 per pixel on an 8 pixel sense grid row
            speed_fraction = joystick_speed * 0.125
            speed = 300 * speed_fraction

            
            if event.direction == "up":
                server.cozmoclient.drive_wheels(lwheel_speed=speed, rwheel_speed = speed, duration=0.3)
            if event.direction == "down":
                server.cozmoclient.drive_wheels(lwheel_speed=-speed, rwheel_speed = -speed, duration=0.3)
            if event.direction == "left":
                server.cozmoclient.drive_wheels(lwheel_speed=-speed, rwheel_speed = speed, duration=0.3)
            if event.direction == "right":
                server.cozmoclient.drive_wheels(lwheel_speed=speed, rwheel_speed = -speed, duration=0.3)

        if connected and joystick_mode == JoystickModes.Lift:
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

        if joystick_mode == JoystickModes.SpheroDrive:
            if len(sphero_devices) > 0 and sphero_attached != sphero_uuid:
                sphero_attaching = True
                sd = sphero_devices.copy()
                if sphero_index > len(sd)-1:
                    sphero_index = len(sd)-1

                attachToSphero(sd[sphero_index])

            else:
                driveParams = {};
                if event.direction == "up":
                    sphero_heading = 0
                    if (sphero_speed < 0):
                        sphero_speed = abs(sphero_speed)
                if event.direction == "down":
                    if (sphero_speed > 0):
                        sphero_speed = -sphero_speed

                    sphero_heading = 0;
                if event.direction == "left":
                    sphero_heading = -1
                if event.direction == "right":
                    sphero_heading = 1
                    
                driveParams["speed"] = sphero_speed
                driveParams["heading"] = sphero_heading
                print("drive: {}".format(driveParams))
                requests.post(sphero_server + "drive", json.dumps(driveParams))

connectLock = Lock()

def powerCycleCozmoCharger():
    print("cutting power to robot")
    requests.post('http://localhost:3001/button-robotbutton', data = {'event':'double-click'})
    sleep(2)
    print("turning power back on")
    requests.post('http://localhost:3001/button-robotbutton', data = {'event':'click'})

def setupRobotInBackground():
    t = Thread(target=setupRobot)
    t.setDaemon(True)
    t.start()

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

        print("setup done.")
    
    except Exception as e:
        print("failed to connect! " + str(e))
        connected = False
        powerCycleCozmoCharger()

    connecting = False
    connectLock.release()
    return connected

sphero_blinky = True
def spheroBlinkThread():
    global sphero_blinky
    while True:
        sleep(0.2)
        sphero_blinky = not sphero_blinky

def displayThread():
    while True:
        update_led()
        sleep(0.1)
        
        
def backgroundSpheroScan():
    spheroscanner = Thread(target=spheroScan)
    spheroscanner.setDaemon(True)
    spheroscanner.start()    
    
def spheroScan():
    global sphero_devices, sphero_scanning, sphero_index
    sphero_devices = []
    sphero_scanning = True
    print("scanning for Sphero devices...")
    res = requests.post(sphero_server + "scan", data = '');
    if res.status_code == 200:
        spheroes = json.loads(res.content)
        print("got scan response: {}".format(spheroes))
        sphero_devices = sorted(spheroes, key=lambda device: device["name"])
        sphero_index = -1
        sphero_scanning = False

def idleSpheroScan():
    global sphero_attached, sphero_attaching, sphero_devices
    if idle > timeout:
        if path.exists("/tmp/live_robots"):
            spheroScan()
            sd = sphero_devices.copy()
            if len(sd) > 0:
                idx = random.randrange(len(sd))
                print("idly connecting to device {}".format(idx))
                attachToSphero(sd[idx])
        elif sphero_attached != None:
            requests.post(sphero_server + "disconnectAll")
            sphero_devices = []
            sphero_attached = None
            sphero_attaching = False

    idleSpheroScanTimer()


def idleSpheroScanTimer():
    spheroScanTimer = Timer(sphero_idlescan_interval, idleSpheroScan)
    spheroScanTimer.setDaemon(True)
    spheroScanTimer.start()

def run():
    global connected

    backgroundSpheroScan()
    idleSpheroScanTimer()
    
    timer = Timer(10, watchdog)
    timer.setDaemon(True)
    timer.start()


    useridletimer = Thread(target=user_idle_watchdog)
    useridletimer.setDaemon(True)
    useridletimer.start()
    
    joysticker = Thread(target=joystickthread)
    joysticker.setDaemon(True)
    joysticker.start()

    blinkThread = Thread(target=spheroBlinkThread)
    blinkThread.setDaemon(True)
    blinkThread.start()

    uiThread = Thread(target=displayThread)
    uiThread.setDaemon(True)
    uiThread.start()
    
    #spheroscanner = Thread(target=spheroScanThread)
    #spheroscanner.setDaemon(True)
    #spheroscanner.start()
    try:
        print("web server listening on " + address[0] + ":" + str(address[1]))
        server.serve_forever()
    finally:
        print("goodbye")
        requests.post(sphero_server + "disconnectAll")
        sense.set_pixels(exitscreen)


output = StreamingOutput()
server = StreamingServer(address, StreamingHandler)
run()
