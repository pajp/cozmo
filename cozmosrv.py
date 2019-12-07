import io
import pycozmo
import logging
import socketserver
from threading import Condition
from PIL import ImageFont, ImageDraw, Image
import cv2
import traceback
import time
import json
import numpy as np
import os
import datetime as dt

#import SimpleHTTPServer

from http.server import BaseHTTPRequestHandler,HTTPServer

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
        print("retain")
        self.clientCount = self.clientCount + 1
        if self.clientCount == 1:
            print("enabling camera")
            self.cozmoclient.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image, one_shot=False)


    def release(self):
        self.clientCount = self.clientCount - 1
        print("release, %d clients remaining" % (self.clientCount))
        if self.clientCount == 0:
            print("no clients left, disabling camera")
            self.cozmoclient.del_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image)
            
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
        contentlength = self.headers['Content-Length']
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
            height = float(data["height"]) * pycozmo.MAX_LIFT_HEIGHT.mm;
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



output = StreamingOutput()
address = ('', 4443)
server = StreamingServer(address, StreamingHandler)
framecount = 0
def on_camera_image(cli, image):
    del cli
    global framecount
    framecount = framecount + 1
    if framecount % 10 == 0:
        image.save(output, "png")

    server.robotstatusdict["last_image_acquisition"] = time.time()

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
                "timestamp" : pkt.timestamp,
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
                "status" : pkt.status }
    if hasattr(server, 'robotstatusdict'):
        server.robotstatusdict.update(newdict);
    else:
        server.robotstatusdict = newdict



def pycozmo_program(cli: pycozmo.client.Client):
    cli.add_handler(pycozmo.event.EvtRobotChargingChange, on_robot_charging)
    #angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
    cli.set_head_angle(0.1)
    server.robotcharging = -1

    pkt = pycozmo.protocol_encoder.EnableCamera(enable=True)
    cli.conn.send(pkt)
    pkt = pycozmo.protocol_encoder.EnableColorImages(enable=True)
    cli.conn.send(pkt)

    cli.conn.add_handler(pycozmo.protocol_encoder.RobotState, on_robot_state, one_shot=False)
    #cli.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image, one_shot=False)
    server.cozmoclient = cli
    output.cozmoclient = cli
    try:
        server.serve_forever()
    finally:
        print("goodbye")



pycozmo.run_program(pycozmo_program)
