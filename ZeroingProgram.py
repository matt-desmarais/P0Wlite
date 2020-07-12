import os, sys, inspect
# realpath() will make the script run, even if you symlink it
cmd_folder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0]))
if cmd_folder not in sys.path:
    sys.path.insert(0, cmd_folder)
# to include modules from a subfolder
cmd_subfolder = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0],"include")))
if cmd_subfolder not in sys.path:
    sys.path.insert(0, cmd_subfolder)

from picamera import Color
import csv
import time
import math
import IMU
import datetime
import picamera
import numpy as np
import cv2
import RPi.GPIO as GPIO
import patterns
import ConfigParser
import smbus
import subprocess 
from subprocess import Popen, PIPE
import glob
import threading
from collections import OrderedDict
from random import *

global Toggle
Toggle = True

global picControl
picControl = True

global infoVisible
infoVisible=True

global takingScreenshot
takingScreenshot = False

zerofile = "/home/pi/P0Wcrosshair/zero.csv"

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      # Complementary filter constant

gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0

IMU.detectIMU()     #Detect if BerryIMUv1 or BerryIMUv2 is connected.
IMU.initIMU()  

magXmin =  159
magYmin =  -482
magZmin =  -2149
magXmax =  1488
magYmax =  677
magZmax =  -1796

buttoncounter = 0

modeList = ["X-Axis", "Y-Axis", "Zoom", "Crosshair", "Save"]
modeSelection = 0

ycenterList = [300, 301, 304, 308, 313, 319, 327, 336, 346, 357, 370, 384, 399, 415, 433]

global roi
global zoomcount
zoomcount=0
roi="0.0,0.0,1.0,1.0"

zooms = {

    'zoom_step' : 0.03,

    'zoom_xy_min' : 0.0,
    'zoom_xy' : 0.0,
    'zoom_xy_max' : 0.4,

    'zoom_wh_min' : 1.0,
    'zoom_wh' : 1.0,
    'zoom_wh_max' : 0.2

}

def get_file_name_pic():  # new
    return datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S.png")


def update_zoom():
    global roi,zoomcount
    #print "Setting camera to (%s, %s, %s, %s)" % (zooms['zoom_xy'], zooms['zoom_xy'], zooms['zoom_wh'], zooms['zoom_wh'])
    roi = str(zooms['zoom_xy'])[:6], str(zooms['zoom_xy'])[:6], str(zooms['zoom_wh'])[:6], str(zooms['zoom_wh'])[:6]
    roi = str(roi)[1:-1]
    roi = re.sub("'","",roi)
    roi = re.sub(" ","",roi)
    print roi
    camera.zoom = (zooms['zoom_xy'], zooms['zoom_xy'], zooms['zoom_wh'], zooms['zoom_wh'])
    #print "Camera at (x, y, w, h) = ", camera.zoom
    print "zoomcount: "+str(zoomcount)

def set_min_zoom():
    zooms['zoom_xy'] = zooms['zoom_xy_min']
    zooms['zoom_wh'] = zooms['zoom_wh_min']

def set_max_zoom():
    zooms['zoom_xy'] = zooms['zoom_xy_max']
    zooms['zoom_wh'] = zooms['zoom_wh_max']

def zoom_out():
    global zoomcount
    if zooms['zoom_xy'] - zooms['zoom_step'] < zooms['zoom_xy_min']:
        set_min_zoom()
	zoomcount=0
    else:
        zooms['zoom_xy'] -= zooms['zoom_step']
        zooms['zoom_wh'] += (zooms['zoom_step'] * 2)
	zoomcount = zoomcount -1
	update_zoom()

def zoom_in():
    global zoomcount
    if zooms['zoom_xy'] + zooms['zoom_step'] > zooms['zoom_xy_max']:
        set_max_zoom()
    else:
        zooms['zoom_xy'] += zooms['zoom_step']
        zooms['zoom_wh'] -= (zooms['zoom_step'] * 2)
	zoomcount = zoomcount +1
	update_zoom()

def get_file_name():  # new
    return datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S.h264")

def get_file_name_pic():  # new
    return datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S.jpg")

#gunRange = 30
alphaValue = 64

Enc_A1 = 17 				# Encoder input A: input GPIO 4 
Enc_B1 = 27  			        # Encoder input B: input GPIO 14 
Button1 = 22
Rotary_counter1 = 0  			# Start counting from 0
Current_A1 = 1					# Assume that rotary switch is not 
Current_B1 = 1					# moving while we init software
LockRotary1 = threading.Lock()		# create lock for rotary switch

globalCounter = 0

flag = 0
Last_RoB_Status = 0
Current_RoB_Status = 0

# subclass for ConfigParser to add comments for settings
# (adapted from jcollado's solution on stackoverflow)
class ConfigParserWithComments(ConfigParser.ConfigParser):
    def add_comment(self, section, comment):
        self.set(section, '; %s' % (comment,), None)

    def write(self, fp):
        """Write an .ini-format representation of the configuration state."""
        for section in self._sections:
            fp.write("[%s]\n" % section)
            for (key, value) in self._sections[section].items():
                self._write_item(fp, key, value)
            fp.write("\n")

    def _write_item(self, fp, key, value):
        if key.startswith(';') and value is None:
            fp.write("%s\n" % (key,))
        else:
            fp.write("%s = %s\n" % (key, str(value).replace('\n', '\n\t')))

# settings from config file:
configfile = '/boot/crosshair.cfg'
cdefaults = {
            'width': '800',
            'height': '600',
            'color': 'white',
            'pattern': '1',
            'radius': '100',
            'xcenter': '400',
            'ycenter': '300',
            'stream': 'false',
            'upload': 'false'
            }


def clear(ev=None):
        globalCounter = 0
	#print 'globalCounter = %d' % globalCounter
	time.sleep(1)

def rotaryClear():
	print("Cleared")
        GPIO.add_event_detect(RoSPin, GPIO.FALLING, callback=clear) # wait for falling

# if config file is missing, recreate it with default values:
def CreateConfigFromDef(fileloc,defaults):
    print "Config file not found."
    print "Recreating " + fileloc + " using default settings."
    config.add_section('main')
    config.add_section('overlay')
    config.set('overlay', 'xcenter', cdefaults.get('xcenter'))
    config.set('overlay', 'ycenter', cdefaults.get('ycenter'))
    config.add_comment('overlay', 'color options: white (default), red, green, blue, yellow')
    config.set('overlay', 'color', cdefaults.get('color'))
    config.add_comment('overlay', 'pattern options:')
    config.add_comment('overlay', '1: Bruker style with circles and ticks')
    config.add_comment('overlay', '2: simple crosshair with ticks')
    config.add_comment('overlay', '3: simple crosshair without ticks')
    config.add_comment('overlay', '4: crosshair with circles, no ticks')
    config.add_comment('overlay', '5: crosshair with one circle, no ticks')
    config.add_comment('overlay', '6: only one circle')
    config.add_comment('overlay', '7: small crosshair')
    config.add_comment('overlay', '8: small crosshair without intersection')
    config.add_comment('overlay', '9: only a dot')
    config.add_comment('overlay', '10: grid')
    config.set('overlay', 'pattern', cdefaults.get('pattern'))
    config.add_comment('overlay', 'set radius (in px) for all circles,')
    config.add_comment('overlay', 'also controls grid spacing in Pattern 10')
    config.set('overlay', 'radius', cdefaults.get('radius'))
    config.set('main', 'width', cdefaults.get('width'))
    config.set('main', 'height', cdefaults.get('height'))
    config.add_comment('main', 'uploading and streaming not implemented yet')
    config.set('main', 'upload', cdefaults.get('upload'))
    config.set('main', 'stream', cdefaults.get('stream'))
    # write default settings to new config file:
    with open(fileloc, 'wb') as f:
        config.write(f)

# try to read settings from config file; if it doesn't exist
# create one from defaults & use same defaults for this run:
try:
    with open(configfile) as f:
        config = ConfigParserWithComments(cdefaults)
        config.readfp(f)
except IOError:
    config = ConfigParserWithComments(cdefaults)
    CreateConfigFromDef(configfile,cdefaults)

# retrieve settings from config parser:
width = int(config.get('main', 'width'))
height = int(config.get('main', 'height'))
print "Set resolution: " + str(width) + "x" + str(height)
# make sure width is a multiple of 32 and height
# is a multiple of 16:
if (width%32) > 0 or (height%16) > 0:
    print "Rounding down set resolution to match camera block size:"
    width = width-(width%32)
    height = height-(height%16)
    print "New resolution: " + str(width) + "x" + str(height)
curcol = config.get('overlay', 'color')
curpat2 = int(config.get('overlay', 'pattern'))
curpat = 1
xcenter = int(config.get('overlay', 'xcenter'))
ycenter = int(config.get('overlay', 'ycenter'))
radius = int(config.get('overlay', 'radius'))

#curpat2 = 1
# map colors:
colors = {
        'white': (255,255,255),
        'red': (255,0,0),
        'green': (0,255,0),
        'blue': (0,0,255),
        'yellow': (255,255,0),
        }

# initialize toggle for on/off button and gui state:
togsw = 1
guivisible = 1


counter = 0

# initialize GPIO and assign buttons:
GPIO.setmode(GPIO.BCM)
GPIO.setup(Enc_A1, GPIO.IN)
GPIO.setup(Enc_B1, GPIO.IN)
GPIO.setup(Button1, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# function to call when top button is pressed (GPIO 24):
def toggleonoff(channel):
    global togsw,o,alphaValue,infoVisible

    #if 'o' in globals() and o != None:
    #    camera.remove_overlay(o)
    #o = None
    
    if infoVisible:
        ovl = np.zeros((height, width, 3), dtype=np.uint8)
        patternswitcher(ovl,0)
        infoVisible=False
        return
    else:
        gui = np.zeros((height, width, 3), dtype=np.uint8)
        #creategui(gui)
        patternswitcher(gui,0)
        infoVisible=True
        return
    return

# function to call when middle button is pressed (GPIO 23):
def togglepattern(var):
    global togsw,o,curpat2,col,ovl,gui,alphaValue
    # if overlay is inactive, ignore button:
    if togsw == 0:
        print "Pattern button pressed, but ignored --- Crosshair not visible."
    # if overlay is active, drop it, change pattern, then show it again
    else:
        curpat2 += var
        print "Set new pattern: " + str(curpat2) 
        if curpat2 > patterns.maxpat:     # this number must be adjusted to number of available patterns!
            curpat2 = 1
        if curpat2 < 1:
            curpat2 = 10
        if guivisible == 0:
            # reinitialize array:
            ovl = np.zeros((height, width, 3), dtype=np.uint8)
            patternswitcher(ovl,0)
            if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(ovl), layer=3, alpha=alphaValue)
        else:
            # reinitialize array
            gui = np.zeros((height, width, 3), dtype=np.uint8)
            #creategui(gui)
            if infoVisible:
                patternswitcher(gui,1)
            elif infoVisible == False:
                patternswitcher(gui,0)
            #patternswitch(gui,1)
            if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(gui), layer=3, alpha=alphaValue)
    return


def togglepattern2(channel):
    global togsw,o,curpat2,col,ovl,gui,alphaValue
    # if overlay is inactive, ignore button:
    if togsw == 0:
        print "Pattern button pressed, but ignored --- Crosshair not visible."
    # if overlay is active, drop it, change pattern, then show it again
    else:
        curpat2 += 1
        print "Set new pattern: " + str(curpat2) 
        if curpat2 > 6:     # this number must be adjusted to number of available patterns!
            curpat2 = 1
        if guivisible == 0:
            # reinitialize array:
            ovl = np.zeros((height, width, 3), dtype=np.uint8)
            patternswitcher(ovl,0)
            if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(ovl), layer=3, alpha=alphaValue)
        else:
            # reinitialize array
            gui = np.zeros((height, width, 3), dtype=np.uint8)
            #creategui(gui)
            if infoVisible:
                patternswitcher(gui,1)
            elif infoVisible == False:
                patternswitcher(gui,0)
            #patternswitcher(gui,1)
            if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(gui), layer=3, alpha=alphaValue)
    return


def togglepattern3():
    global togsw,o,curpat,col,ovl,gui,alphaValue,ycenter,zoomcount
    # if overlay is inactive, ignore button:
    if togsw == 0:
        print "Pattern button pressed, but ignored --- Crosshair not visible."
    # if overlay is active, drop it, change pattern, then show it again
    else:
        if guivisible == 0:
	    # reinitialize array:
            ovl = np.zeros((height, width, 3), dtype=np.uint8)
            patternswitcherZoomIn(ovl,0)
	    if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(ovl), layer=3, alpha=alphaValue)
	else:
            # reinitialize array
	    gui = np.zeros((height, width, 3), dtype=np.uint8)
	    creategui(gui)
            patternswitcherZoomIn(gui,1)
            if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(gui), layer=3, alpha=alphaValue)
    return


# function 
def togglepatternZoomIn():
    global togsw,o,curpat,col,ovl,gui,alphaValue,ycenter,zoomcount
    # if overlay is inactive, ignore button:
    if togsw == 0:
        print "Pattern button pressed, but ignored --- Crosshair not visible."
	zoom_in()
	#if ycenter < cdefaults.get('ycenter')
	ycenter = ycenterList[zoomcount]
    # if overlay is active, drop it, change pattern, then show it again
    else:
        if guivisible == 0:
            zoom_in()
	    # reinitialize array:
            ovl = np.zeros((height, width, 3), dtype=np.uint8)
            patternswitcherZoomIn(ovl,0)
	    if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(ovl), layer=3, alpha=alphaValue)
	else:
            # reinitialize array
            zoom_in()
	    gui = np.zeros((height, width, 3), dtype=np.uint8)
	    #creategui(gui)
            patternswitcherZoomIn(gui,1)
            if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(gui), layer=3, alpha=alphaValue)
    return

# function to call when middle button is pressed (GPIO 23):
def togglepatternZoomOut():
    global togsw,ycenter,o,curpat,col,ovl,gui,alphaValue,zoomcount,ycenterList
    # if overlay is inactive, ignore button:
    #ycenter = ycenterList[zoomcount]
    print zoomcount
    #print "shit balls"
    if togsw == 0:
        zoom_out()
	print "Pattern button pressed, but ignored --- Crosshair not visible."
    # if overlay is active, drop it, change pattern, then show it again
    else:
        if guivisible == 0:
	    zoom_out()
            # reinitialize array:
            ovl = np.zeros((height, width, 3), dtype=np.uint8)
            patternswitcherZoomOut(ovl,0)
            if 'o' in globals() and o != None:
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(ovl), layer=3, alpha=alphaValue)
        else:
	    zoom_out()
            # reinitialize array
            gui = np.zeros((height, width, 3), dtype=np.uint8)
            #creategui(gui)
            patternswitcherZoomOut(gui,1)
            if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(gui), layer=3, alpha=alphaValue)
    return


def patternswitcherZoomIn(target,guitoggle):
    global o, zoomcount, ycenter
    # first remove existing overlay:
    if 'o' in globals() and o != None:
        camera.remove_overlay(o)
    if guitoggle == 1:
        creategui(gui)
    if zooms['zoom_xy'] == zooms['zoom_xy_max']:
	print("zoom at max")
    # cycle through possible patterns:
    if curpat2 == 1:
        patterns.pattern2(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 2:
        patterns.pattern3(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 3:
        patterns.pattern5(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 4:
        patterns.pattern6(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 5:
        patterns.pattern7(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 6:
        patterns.pattern8(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 7:
        patterns.pattern8(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 8:
        patterns.pattern8(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 9:
        patterns.pattern9(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 10:
        patterns.pattern10(target, width, height, xcenter, ycenter, radius, col)
    #if guitoggle == 1:
    #    creategui(gui)
    # Add the overlay directly into layer 3 with transparency;
    # we can omit the size parameter of add_overlay as the
    # size is the same as the camera's resolution
    o = camera.add_overlay(np.getbuffer(target), layer=3, alpha=alphaValue)
    #cv2.imwrite('/home/pi/messigray.png', np.getbuffer(gui))
    return

def patternswitcherZoomOut(target,guitoggle):
    global o, zoomcount, ycenter, ycenterList
    # first remove existing overlay:
    if 'o' in globals() and o != None:
        camera.remove_overlay(o)
####    if guitoggle == 1:
####        creategui(gui)
    if zooms['zoom_xy'] == zooms['zoom_xy_min']:
        print("zoom at min")
    # cycle through possible patterns:
    if curpat2 == 1:
        patterns.pattern2(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 2:
        patterns.pattern3(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 3:
        patterns.pattern5(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 4:
        patterns.pattern6(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 5:
        patterns.pattern7(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 6:
        patterns.pattern8(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 7:
        patterns.pattern8(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 8:
        patterns.pattern8(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 9:
        patterns.pattern9(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 10:
        patterns.pattern10(target, width, height, xcenter, ycenter, radius, col)
#    if guitoggle == 1:
#        creategui(gui)
    # Add the overlay directly into layer 3 with transparency;
    # we can omit the size parameter of add_overlay as the
    # size is the same as the camera's resolution
    o = camera.add_overlay(np.getbuffer(target), layer=3, alpha=alphaValue)
    return


# function to call when low button is pressed (GPIO 18):
def togglecolor(channel):
    global togsw,o,curcol,col,ovl,gui,alphaValue,infoVisible
    # step up the color to next in list
    curcol = colorcycle(colors,curcol)
    # map colorname to RGB value for new color
    col = colormap(curcol)
    # if overlay is inactive, ignore button:
    if togsw == 0:
        print "Color button pressed, but ignored --- Crosshair not visible."
    # if overlay is active, drop it, change color, then show it again
    else:
        print "Set new color: " + str(curcol) + "  RGB: " + str(col) 
        if guivisible == 0:
            # reinitialize array:
            ovl = np.zeros((height, width, 3), dtype=np.uint8)
            patternswitcher(ovl,0)
            if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(ovl), layer=3, alpha=alphaValue)
        else:
            # reinitialize array
            gui = np.zeros((height, width, 3), dtype=np.uint8)
            #creategui(gui)
            if infoVisible:
                patternswitcher(gui,1) 
            elif infoVisible == False:
                patternswitcher(gui,0) 
            if 'o' in globals():
                camera.remove_overlay(o)
            o = camera.add_overlay(np.getbuffer(gui), layer=3, alpha=alphaValue)
    return


def takeScreenshot(channel):
	global takingScreenshot
	if takingScreenshot == False:
	    subprocess.Popen("./raspi2png",  cwd='/home/pi/raspi2png')
	    takingScreenshot = True
	    time.sleep(20)
	    #generate filename for dropbox file name
	    filename = get_file_name_pic()

	    photofile = "/home/pi/Dropbox-Uploader/dropbox_uploader.sh upload /home/pi/raspi2png/snapshot.png /PiGun/"+filename
	    #runs photofile dropbox upload
	    subprocess.Popen(photofile, shell=True)
	    takingScreenshot = False

def rotary_interrupt1(A_or_B):
	global Rotary_counter1, Current_A1, Current_B1, LockRotary1
													# read both of the switches
	Switch_A1 = GPIO.input(Enc_A1)
	Switch_B1 = GPIO.input(Enc_B1)
													# now check if state of A or B has changed
													# if not that means that bouncing caused it
	if Current_A1 == Switch_A1 and Current_B1 == Switch_B1:		# Same interrupt as before (Bouncing)?
		return										# ignore interrupt!

	Current_A1 = Switch_A1								# remember new state
	Current_B1 = Switch_B1								# for next bouncing check


	if (Switch_A1 and Switch_B1):						# Both one active? Yes -> end of sequence
		LockRotary1.acquire()						# get lock 
		if A_or_B == Enc_B1:							# Turning direction depends on 
			Rotary_counter1 += 1						# which input gave last interrupt
		else:										# so depending on direction either
			Rotary_counter1 -= 1						# increase or decrease counter
		LockRotary1.release()						# and release lock
	return											# THAT'S IT


def markClear(channel):
	d = OrderedDict()
	d['value'] = 23
	d['lat'] = gpsd.fix.latitude
	d['lon'] = gpsd.fix.longitude
	d['ele'] = gpsd.fix.altitude
	print "dump:",json.dumps(d)
	client.publish('clearlocations',json.dumps(d))	



def writeZeroFile(zoom, yaxis):
    with open(zerofile, 'a') as file:
        writer = csv.writer(file)
        writer.writerow([zoom, yaxis])

def toggleXY(channel):
	global Toggle, zoomcount, ycenter
	if Toggle == True:
	    Toggle = False
	else:
	    Toggle = True 
        if(modeSelection == 4):
            writeZeroFile(zoomcount, ycenter)

GPIO.setmode(GPIO.BCM)
GPIO.add_event_detect(22, GPIO.FALLING, callback=toggleXY, bouncetime=300)

GPIO.add_event_detect(Enc_A1, GPIO.RISING, callback=rotary_interrupt1) 				# NO bouncetime 
GPIO.add_event_detect(Enc_B1, GPIO.RISING, callback=rotary_interrupt1) 			# NO bouncetime 


# map text color names to RGB:
def colormap(col):
    return colors.get(col, (255,255,255))    # white is default

# cycle through color list starting from current color:
def colorcycle(self, value, default='white'):
    # create an enumerator for the entries and step it up
    for i, item in enumerate(self):
        if item == value:
            i += 1
            # if end of color list is reached, jump to first in list
            if i >= len(self):
                i = 0
            return self.keys()[i]
    # if function fails for some reason, return white
    return default

# function to construct/draw the GUI
#def creategui(target):
    #cv2.putText(target, gui1, (10,height-108), font, 2, col, 2)
    #cv2.putText(target, gui2, (10,height-78), font, 2, col, 2)
    #cv2.putText(target, gui3, (10,height-78), font, 2, col, 2)
    #cv2.putText(target, ' Lat:'+str(gpsd.fix.latitude), (10,height-48), font, 2, col, 2)
    #cv2.putText(target, ' Long:'+str(gpsd.fix.longitude), (10,height-18), font, 2, col, 2)
    #cv2.putText(target, 'GUI will vanish after 10s', (10,30), font, 2, col, 2)
#    return

############################################################

def patternswitcher(target,guitoggle):
    global o
    # first remove existing overlay:
    if 'o' in globals():
        camera.remove_overlay(o)
####    if guitoggle == 1:
####        creategui(gui)
    # cycle through possible patterns:
    if curpat2 == 1:
        patterns.pattern1(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 2:
        patterns.pattern2(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 3:
        patterns.pattern3(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 4:
        patterns.pattern4(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 5:
        patterns.pattern5(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 6:
        patterns.pattern6(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 7:
        patterns.pattern7(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 8:
        patterns.pattern8(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 9:
        patterns.pattern9(target, width, height, xcenter, ycenter, radius, col)
    if curpat2 == 10:
        patterns.pattern10(target, width, height, xcenter, ycenter, radius, col)
    # Add the overlay directly into layer 3 with transparency;
    # we can omit the size parameter of add_overlay as the
    # size is the same as the camera's resolution
    o = camera.add_overlay(np.getbuffer(target), layer=3, alpha=alphaValue)
    return


############################################################

# create array for the overlay:
ovl = np.zeros((height, width, 3), dtype=np.uint8)
font = cv2.FONT_HERSHEY_PLAIN
col = colormap(curcol)
# create array for a bare metal gui and text:
gui = np.zeros((height, width, 3), dtype=np.uint8)
gui1 = ' Airsoft Landwarrior'
gui2 = ' Version 1.0 Beta'
#gui3 = ' button  = cycle distance'
#gui4 = ' range: '+str(gunRange)
#gui5 = ' s/r     = save/revert settings'



with picamera.PiCamera() as camera:
#    global Rotary_counter1, LockRotary, Toggle
    NewCounter1 = 0
    camera.resolution = (width, height)
    camera.framerate = 24
    filename = get_file_name()
    camera.meter_mode='matrix'
    #####RECORDING LINE BELOW
    camera.start_recording(filename)
    # set this to 1 when switching to fullscreen output
    camera.preview_fullscreen = 1
    camera.start_preview()
    try:
        patternswitcher(gui,1)
        time.sleep(2)
        guivisible = 1
        togglepatternZoomOut()
        a = datetime.datetime.now()
	while True:
		LockRotary1.acquire()					# get lock for rotary switch
		NewCounter1 = Rotary_counter1		# get counter value
		Rotary_counter1 = 0						# RESET IT TO 0
		LockRotary1.release()					# and release lock
                if(modeSelection == 0):
                    value = str(xcenter)
                if(modeSelection == 1):
                    value = str(ycenter)
                if(modeSelection == 2):
                    value = str(zoomcount)
                if(modeSelection == 3):
                    value = str(curpat2)
                if(modeSelection == 4):
                    value = "Z:"+str(zoomcount)+" Y:"+str(ycenter)
		if (NewCounter1 !=0):					# Counter has CHANGED
			print NewCounter1		# some test print
			if(NewCounter1 > 0):
				for x in range(0,NewCounter1):
					if Toggle and modeSelection < len(modeList)-1:
						modeSelection = modeSelection + 1
					elif Toggle:
						modeSelection = 0
                                        else:
                                            if(modeSelection == 0):
                                                xcenter = xcenter +1
                                            if(modeSelection == 1):
                                                ycenter = ycenter +1
                                            if(modeSelection == 2):
                                                if zooms['zoom_xy'] != zooms['zoom_xy_max']:
                                                    togglepatternZoomIn()
                                            if(modeSelection == 3):
                                                togglepattern(1)
				togglepattern3()
			else:
				for x in range(0,abs(NewCounter1)):
					if Toggle and modeSelection > 0:
                                                modeSelection = modeSelection - 1
                                        elif Toggle:
                                                modeSelection = len(modeList)-1
                                        else:
                                            if(modeSelection == 0):
                                                xcenter = xcenter -1
                                            if(modeSelection == 1):
                                                ycenter = ycenter -1
                                            if(modeSelection == 2):
                                                if zooms['zoom_xy'] != zooms['zoom_xy_min']:
                                                    togglepatternZoomOut()
                                            if(modeSelection == 3):
                                                togglepattern(-1)
				togglepattern3()

		   #Read the gyroscope and magnetometer values
    		ACCx = IMU.readACCx()
		ACCy = IMU.readACCy()
		ACCz = IMU.readACCz()
		GYRx = IMU.readGYRx()
  		GYRy = IMU.readGYRy()
  		GYRz = IMU.readGYRz()
  	  	MAGx = IMU.readMAGx()
   	 	MAGy = IMU.readMAGy()
		MAGz = IMU.readMAGz()

                time.sleep(.15)

                ACCx2 = IMU.readACCx()
                ACCy2 = IMU.readACCy()
                ACCz2 = IMU.readACCz()

                total1 = ACCx + ACCy + ACCz

	   	MAGx -= (magXmin + magXmax) /2
    		MAGy -= (magYmin + magYmax) /2
    		MAGz -= (magZmin + magZmax) /2


                total2 = ACCx2 + ACCy2 + ACCz2

                #print("total1: "+str(total1))
                #print("total2: "+str(total2))


                #print("total diff: "+str(abs(total1-total2)/2))

                diff = abs(total1-total2)/2

                if  (diff < 150):
                #    print("low")
                    stable = "|   |"
                elif  (diff >= 150 and diff < 500):
                #    print("med")
                    stable = "| | | |       | | | |"
                elif  (diff >= 500):
                #    print("high")
                    stable = "|-|-|-|       |-|-|-|"


		b = datetime.datetime.now() - a
		a = datetime.datetime.now()
		LP = b.microseconds/(1000000*1.0)
		#print "Loop Time | %5.2f|" % ( LP ),

		#Convert Gyro raw to degrees per second
		rate_gyr_x =  GYRx * G_GAIN
		rate_gyr_y =  GYRy * G_GAIN
		rate_gyr_z =  GYRz * G_GAIN

		#Calculate the angles from the gyro. 
		gyroXangle+=rate_gyr_x*LP
		gyroYangle+=rate_gyr_y*LP
		gyroZangle+=rate_gyr_z*LP

		#Calculate heading
		heading = 180 * math.atan2(MAGy,MAGx)/M_PI

		#Only have our heading between 0 and 360
		if heading < 0:
			heading += 360

		#Normalize accelerometer raw values.
		accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
		accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
		#Calculate pitch and roll
		try:
		    pitch = math.asin(accXnorm)
		    #print "Pitch: "+str(pitch)
		    roll = -math.asin(accYnorm/math.cos(pitch))
		    #print "RollL "+str(roll)
		    #Calculate the new tilt compensated values
		    magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
		    magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)

		    #Calculate tilt compensated heading
		    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
		except ValueError:
		    print "Value Error"
		camera.annotate_text_size = 70
                camera.annotate_text = str(modeList[modeSelection])+": "+value+"\n\n\n"+stable+"\n"+stable+"\n\n\nPitch:"+str(pitch)[0:5]+" Roll:"+str(roll)[0:5]
    except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
        print "\nExiting Program"
    finally:
        camera.close()               # clean up camera
        GPIO.cleanup()               # clean up GPIO
