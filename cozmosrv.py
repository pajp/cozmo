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

import datetime as dt

#import SimpleHTTPServer

from http.server import BaseHTTPRequestHandler,HTTPServer

PAGE="""\
<html>
<head>
<title>cozmoctrl</title>
</head>
<body>
<h1>cozmo!</h1>
<img src="stream.mjpg" />
</body>
</html>
"""

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

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
            
        if self.path == '/tilt':
            angle = int(input)
            print("setting head angle to %d" % angle);
            self.server.cozmoclient.set_head_angle(angle)
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
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/forward':
            self.server.cozmoclient.drive_wheels(lwheel_speed=100, rwheel_speed = 100, duration=1.0)
            self.send_response(200)
            self.end_headers()

        elif self.path == '/back':
            self.server.cozmoclient.drive_wheels(lwheel_speed=-100, rwheel_speed = -100, duration=1.0)
            self.send_response(200)
            self.end_headers()

        elif self.path == '/turnleft':
            self.server.cozmoclient.drive_wheels(lwheel_speed=-100, rwheel_speed = 100, duration=0.25)
            self.send_response(200)
            self.end_headers()

        elif self.path == '/turnright':
            self.server.cozmoclient.drive_wheels(lwheel_speed=100, rwheel_speed = -100, duration=0.25)
            self.send_response(200)
            self.end_headers()

        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        #print("waiting for condition")
                        output.condition.wait()
                        frame = output.frame
                        # now add timestamp to jpeg
                        # Convert to PIL Image
                        cv2.CV_LOAD_IMAGE_COLOR = 1 # set flag to 1 to give colour image
                        #npframe = np.fromstring(frame, dtype=np.uint8)
                        #pil_frame = 
                        #pil_frame = cv2.imdecode(frame,-1)
                        #cv2_im_rgb = cv2.cvtColor(pil_frame, cv2.COLOR_BGR2RGB)
                        pil_im = Image.open(io.BytesIO(frame))

                        draw = ImageDraw.Draw(pil_im)

                        # Choose a font
                        #font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 25)
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
                        # WeatherSTEM logo in lower left
                        #size = 64
                        #WSLimg = Image.open("WeatherSTEMLogoSkyBackground.png")
                        #WSLimg.thumbnail((size,size),Image.ANTIALIAS)
                        #pil_im.paste(WSLimg, (0, bg_h-size))

                        # SkyWeather log in lower right
                        #SWLimg = Image.open("SkyWeatherLogoSymbol.png")
                        #SWLimg.thumbnail((size,size),Image.ANTIALIAS)
                        #pil_im.paste(SWLimg, (bg_w-size, bg_h-size))

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
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True



#with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
#with picamera.PiCamera(resolution='1920x1080', framerate=24) as camera:

#----

output = StreamingOutput()
address = ('', 4443)
server = StreamingServer(address, StreamingHandler)
framecount = 0
def on_camera_image(cli, image):
    del cli
    global framecount
    #print("saving image")
    framecount = framecount + 1
    if framecount % 10 == 0:
        image.save(output, "png")

def on_robot_charging(cli, state):
    del cli
    if state:
        print("Started charging.")
        server.robotcharging = 1
    else:
        print("Stopped charging.")
        server.robotcharging = 0        


        
def on_robot_state(cli, pkt: pycozmo.protocol_encoder.RobotState):
    del cli
    server.robotstatus = "Batt: {:.01f} V".format(pkt.battery_voltage)
    #print("-> ", server.robotstatus)


def pycozmo_program(cli: pycozmo.client.Client):
    cli.add_handler(pycozmo.event.EvtRobotChargingChange, on_robot_charging)
    angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
    cli.set_head_angle(angle)
    server.robotcharging = -1

    pkt = pycozmo.protocol_encoder.EnableCamera(enable=True)
    cli.conn.send(pkt)
    pkt = pycozmo.protocol_encoder.EnableColorImages(enable=True)
    cli.conn.send(pkt)

    # Wait for image to stabilize.
    time.sleep(2.0)

    cli.conn.add_handler(pycozmo.protocol_encoder.RobotState, on_robot_state, one_shot=False)
    cli.add_handler(pycozmo.event.EvtNewRawCameraImage, on_camera_image, one_shot=False)
    server.cozmoclient = cli
    
    try:
        server.serve_forever()
    finally:
        print("goodbye")



pycozmo.run_program(pycozmo_program)




#-----


# with picamera.PiCamera(resolution='1296x730', framerate=24) as camera:
#     output = StreamingOutput()
#     camera.start_recording(output, format='mjpeg')
#     camera.annotate_foreground = picamera.Color(y=0.2,u=0, v=0)
#     camera.annotate_background = picamera.Color(y=0.8, u=0, v=0)
#     try:
#         address = ('', 443)
#         server = StreamingServer(address, StreamingHandler)
#         server.serve_forever()
#     finally:
#         camera.stop_recording()
