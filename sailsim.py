#!/usr/bin/python
from __future__ import absolute_import, division, print_function, unicode_literals

import RPi.GPIO as GPIO
import random, glob, time

from math import cos, acos, sin, asin, radians, degrees, pi, atan2, hypot
from Noise3D import Noise3D

import sys
sys.path.append("/home/pi/pi3d")
import pi3d

# GPIO input
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_UP)
R_POT = 0
LAST = "00"
STATE = {"0001":1, "0010":-1, "0100":-1, "0111":1,
         "1000":1, "1011":-1, "1101":-1, "1110":1}

def rudder_callback(chan):
  global R_POT, LAST, STATE
  curr = str(GPIO.input(23)) + str(GPIO.input(24))
  key = LAST + curr
  if key in STATE:
    drctn = STATE[key]
    LAST = curr
    R_POT += drctn

GPIO.add_event_detect(24, GPIO.BOTH, callback=rudder_callback)
GPIO.add_event_detect(23, GPIO.BOTH, callback=rudder_callback)

# Setup display and initialise pi3d
DISPLAY = pi3d.Display.create(x=50, y=50, frames_per_second=20)
DISPLAY.set_background(0.4, 0.6, 0.8, 1.0)      # r,g,b,alpha
# yellowish directional light blueish ambient light
LIGHT = pi3d.Light(lightpos=(1, -1, -3), lightcol =(1.0, 1.0, 0.8), lightamb=(0.25, 0.2, 0.3))
#create shaders
shader = pi3d.Shader("uv_reflect")
flatsh = pi3d.Shader("uv_flat")
matsh = pi3d.Shader("mat_reflect")
# cameras
CAMERA = pi3d.Camera((0,0,0), (0,0,-0.1), (1.0, 1000, 60, DISPLAY.width/float(DISPLAY.height)))
CAM2D = pi3d.Camera(is_3d=False)

WRITEFILE = False
SHEETMIN, SHEETMAX = 5, 100 # degrees
RUDDERMIN, RUDDERMAX = -pi/2.0, pi/2.0 # arbitrary scale
RPOTMIN, RPOTMAX = -20, 20
#SPOTMIN, SPOTMAX = 2.0, -2.0 # sheet input abitrary scale would be set by calibration procedure
SPOTMIN, SPOTMAX = 0, 50.0 # sheet input abitrary scale would be set by calibration procedure
SPOTFACTOR = 50.0
HIKEMIN, HIKEMAX = 2.0, -2.0 # sheet input abitrary scale would be set by calibration procedure
LARD = 200.0 # hiking moment in arbrary units
ROLLFACTOR = 1.25
WDIRMIN, WDIRMAX = 5.0, 7.0 # wind direction radians
WSTRMIN, WSTRMAX = 3, 25 # wind strength m per sec
POSSTEP = 2.0 #time between updates to boat1 position
DWO = 0.005 #water offset factor to convert uv scale to Sprite scale
DT = 0.1 #time between frames for water
# lookup tables for interpolate to use
# [0][] -> effective area, [1][] -> lift/drag as an angle, [2][] image for sail
SAILINFO = [[1.00,2.16,3.50,4.95,6.06,6.76,7.00,6.76,6.06,4.95,3.50,1.81,0.0],
            [pi,0.6*pi,0.54*pi,0.58*pi,0.6*pi,0.95*pi,pi,1.05*pi,
            1.1*pi,1.25*pi,1.4*pi,1.45*pi,1.49*pi],
            [0,1,2,3,3,3,3,3,3,4,4,4,4]]
# [0][] -> effective area, [1][] -> lift/drag as an angle
RUDDERINFO = [[0.01, 0.03, 0.06, 0.08, 0.1, 0.11], 
            [pi,0.52*pi,0.55*pi,0.6*pi, 0.7*pi, 0.95*pi]]

# environment class containing relevant variables
class Environment(object):
  def __init__(self):
    self.tm = time.time() # time now s
    self.dTm = 0.25 # time between each re-check of conditions s
    self.avWDir = 1.13 # average wind direction radians
    self.nextUp = self.tm + self.dTm
    self.startTm = self.tm
    self.nextPos = self.startTm
    if WRITEFILE:
      self.posFile = open("models/posFile.txt", "w")
    else:
      self.posFile = open("models/posFile.txt", "r")
    self.compx, self.compz, self.comph, self.comps = 0.0, 0.0, 0.0, 0.5
    self.prevx, self.prevz, self.prevh, self.prevs = 0.0, 0.0, 0.0, 0.5
    # 3D Perlin noise generators for direction and strength
    self.noiseDir = Noise3D(128, 1/32.0, 5)
    self.noiseStr = Noise3D(128, 1/32.0, 5, 7)

# boat class with associated variables and methods
class Boat(object):
  def __init__(self):
    self.xm, self.ym, self.zm = 0.0, 0.0, 0.0
    self.heading = 40 # degrees - confusingly oposite sense to compass anticlockwise +ve!
    self.speed = 2 # m per second
    self.heel = 0 # degrees
    self.pitch = 0 # degrees
    self.roll = 0 # rate of change of heel
    self.sheet = 0 # degrees
    self.hike = 0 # -1.0 to 1.0
    self.rudder = 0 # arbitrary scale
    self.rFactor = 0.2 #rudder factor to reduce initial movement
    self.sPot = 4 # sheet input abitrary scale would be set by calibration procedure
    self.Iturn, self.wt = 30, 0.0 # turning inertia, rate of turn degrees/second

    self.hull = pi3d.Model(file_string="models/hull.obj", camera=CAMERA)
    self.hull.set_shader(shader)

    self.sImg = [] # 5x4 png images TODO use normal maps
    for i in range(5):
        self.sImg.append([])
        for j in range(1,5):
            self.sImg[i].append(pi3d.Texture("models/sail"+str(i)+"0"+str(j)+".png"))
    self.sailSeq = 0

    self.sail = pi3d.Model(file_string="models/sail.obj", camera=CAMERA)
    self.sail.set_shader(shader)
    self.sail.set_normal_shine(self.sImg[0][0], 1.0)

  #motion control
  def turnRight(self, step=0.05):
    if ((self.rudder - step) > RUDDERMIN):
      self.rudder -= step
     
  def turnLeft(self, step=0.05):
    if ((self.rudder + step) < RUDDERMAX):
      self.rudder += step
      
  def takeIn(self, step=0.5):
    if self.sPot > SPOTMIN:
      self.sPot -= step
      
  def letOut(self, step=0.5):
    if (self.sPot < SPOTMAX):
      self.sPot += step

  # manage the physics of winds and forces
  def updateVariables(self, e, burgee, burgee2, tiller):
    #tiller.rotateToY(180 - degrees(rudder))
    self.sheet = interpolate([SHEETMIN, SHEETMAX], self.sPot, 0, SPOTFACTOR) * (1 if (self.sheet >= 0) else -1)
    wDir = interpolate([WDIRMIN, WDIRMAX], e.noiseDir.generate((int(self.xm/8)) % 128,
                      (int(self.zm/8)) % 128, int((e.tm - e.startTm)/8.0) % 128), -1.0, 1.0)
    e.avWDir = 0.025*wDir + 0.975 * e.avWDir
    wStr = interpolate([WSTRMIN, WSTRMAX], e.noiseStr.generate((int(self.xm/8)) % 128,
                      (int(self.zm/8)) % 128, int((e.tm - e.startTm)/8.0) % 128), -1.0, 1.0)
    # burgee
    v1 = sin(wDir - radians(self.heading))*wStr + self.roll * ROLLFACTOR
    v2 = cos(wDir - radians(self.heading))*wStr + self.speed
    wDirRel = atan2(v1, v2)
    wStrRel = hypot(v1, v2)
    if (wDirRel > pi):
      wDirRel = wDirRel - 2*pi
    burgee.rotateToZ(degrees(wDirRel))
    burgee.scale(wStrRel/20.0, wStrRel/20.0, 1.0)
    burgee2.rotateToZ(degrees(wDir))
    burgee2.scale(wStr/20.0, wStr/20.0, 1.0)

    if (abs(wDirRel) < abs(radians(self.sheet))): # flogging sail
      self.sheet = degrees(wDirRel)
    angAtt = wDirRel - radians(self.sheet)
    if (self.sheet > 0 and wDirRel < 0): 
      angAtt += 2*pi
    if (self.sheet < 0 and wDirRel > 0): 
      angAtt -= 2*pi
    if (abs(angAtt) > 3.0): #gybe
      self.sheet *= -1
    area = interpolate(SAILINFO[0], abs(angAtt), 0, pi)
    LDangle = interpolate(SAILINFO[1], abs(angAtt), 0, pi)
    sailNum = int(interpolate(SAILINFO[2], abs(angAtt), 0, pi))
    self.sailSeq = (self.sailSeq + 1) % 4
    self.sail.set_normal_shine(self.sImg[sailNum][self.sailSeq], 1.0)

    F = 0.2 * wStrRel * wStrRel * area * cos(radians(self.heel))
    Fheel = F*sin(wDirRel - LDangle*(1 if (wDirRel > 0) else -1))
    self.roll = ((Fheel - self.hike * LARD)/20 - self.heel) / 4.0 # TODO sitting out force to balance
    self.heel += self.roll
    Fdrive = F*cos(wDirRel - LDangle*(1 if (wDirRel > 0) else -1))

    rWt = radians(self.wt)
    rudderRel = atan2(1.0*rWt, self.speed) - self.rudder
    rArea = interpolate(RUDDERINFO[0], abs(rudderRel), 0, RUDDERMAX)
    rLDangle = interpolate(RUDDERINFO[1], abs(rudderRel), 0, RUDDERMAX)
    rF = 10.0 * self.speed * self.speed * rArea #TODO rotational damping needed
    rFdrag = -rF * cos(rLDangle)
    rFturn = rF * sin(rLDangle) * (1 if (rudderRel > 0) else -1) - self.rudder # to get out of irons!

    hullDrag = 0.1 * self.speed * self.speed * self.speed * (1 if (self.speed > 0) else -1)
    windage = cos(wDirRel) * wStrRel
    windage = -0.5 * windage * windage*(1 if (abs(wDirRel) > pi/2.0) else -1)
    hullDrag += windage
    
    acc = (Fdrive - rFdrag - hullDrag)/200
    self.speed += acc * e.dTm
    if self.speed < 0.5:
      self.speed = 0.5
    elif self.speed > 8.0:
      self.speed = 8.0
        
    #TODO the height up the mast for turning force needs to be a static var
    wtDot = (-rFturn - 50*rWt*rWt*(1 if (rWt > 0) else -1) - Fdrive*0.05*sin(radians(self.heel)))/self.Iturn
    self.wt += degrees(wtDot * e.dTm)
    if self.wt > 20.0:
      self.wt = 20.0 
    elif self.wt < -20.0:
      self.wt = -20.0 

# Load textures
ectex = pi3d.loadECfiles("textures/ecubes", "sbox")
myecube = pi3d.EnvironmentCube(size=900.0, maptype="FACES", camera=CAMERA)
myecube.set_draw_details(flatsh, ectex)

# water cards, normal images for animating water surace
wimg = []
iFiles = glob.glob("textures/water/n_norm???.png")
iFiles.sort() # order is vital to animation!
for f in iFiles:
  wimg.append(pi3d.Texture(f))
nImg = len(wimg)
shapeshine = pi3d.Texture("textures/stars.jpg")

tSize = 120.0
water = pi3d.Sprite(w=tSize, h=tSize, camera=CAMERA)
water.set_draw_details(matsh, [wimg[0], shapeshine], 12.0, 0.6)
water.set_material((0.0, 0.05, 0.1))
water.set_fog((0.4, 0.6, 0.8, 0.0),150)
water.rotateToX(89.9999)

# marks
mark1 = pi3d.Cylinder(radius=0.25, height=15, x=0.0, y=-5.0, z=450.0, camera=CAMERA)
mark1.set_draw_details(matsh, [wimg[0], shapeshine], 0.2, 0.2)
mark1.set_material((1.0, 0.0, 0.0)) # red

mark2 = pi3d.Cylinder(radius=0.25, height=15, x=-250.0, y=-5.0, z=225.0, camera=CAMERA)
mark2.set_draw_details(matsh, [wimg[0], shapeshine], 0.2, 0.2)
mark2.set_material((1.0, 1.0, 1.0)) # white

mark3 = pi3d.Cylinder(radius=0.25, height=15, x=-10.0, y=-5.0, z=10.0, camera=CAMERA)
mark3.set_draw_details(matsh, [wimg[0], shapeshine], 0.2, 0.2)
mark3.set_material((0.0, 0.0, 1.0)) # and blue

dw, dh = DISPLAY.width, DISPLAY.height
# relative wind direction indicator
burgee = pi3d.ImageSprite(texture="models/burgee.png", shader=flatsh,
                  w=64, h=64, camera=CAM2D)
burgee.position(0.0, -dh/2.0 + 96, 1.0)

# absolute wind direction indicator in middle of map
burgee2 = pi3d.ImageSprite(texture="models/burgee.png", shader=flatsh,
                  w=64, h=64, camera=CAM2D)
burgee2.position(dw/2.0 - 75.0, -dh/2.0 + 75, 1.0)

# tiller
tiller = pi3d.ImageSprite(texture="models/tiller.png", shader=flatsh,
                  w=64, h=64, camera=CAM2D)
tiller.position(0.0, -dh/2.0 + 32, 1.0)

# mark images for map
mktex = pi3d.Texture("models/mark.png")
mk1 = pi3d.ImageSprite(texture=mktex, shader=flatsh, w=16, h=16, camera=CAM2D)
mk1.position(dw/2.0 - 150.0 + mark1.x()/2.0, -dh/2.0 + 25.0 + mark1.z()/2.0, 1.0)
mk2 = pi3d.ImageSprite(texture=mktex, shader=flatsh, w=16, h=16, camera=CAM2D)
mk2.position(dw/2.0 - 150.0 + mark2.x()/2.0, -dh/2.0 + 25.0 + mark2.z()/2.0, 1.0)
mk3 = pi3d.ImageSprite(texture=mktex, shader=flatsh, w=16, h=16, camera=CAM2D)
mk3.position(dw/2.0 - 150.0 + mark3.x()/2.0, -dh/2.0 + 25.0 + mark3.z()/2.0, 1.0)

# boat image for map
boat = pi3d.ImageSprite(texture="models/boat.png", shader=flatsh,
                  w=16, h=16, camera=CAM2D)
boat1 = pi3d.ImageSprite(texture="models/boat.png", shader=flatsh,
                  w=16, h=16, camera=CAM2D) #TODO different colour version of this image

def interpolate(valList, valIn, valMin, valMax):
  n = len(valList)
  val = (float)(valIn - valMin)/(valMax - valMin)*(n - 1.0)
  i = int(val)
  if (val <= 0):
    return valList[0]
  elif (val >= (n-1)):
    return valList[n-1]
  else:
    return (valList[i] + (valList[i+1] - valList[i])*(val - i))

# create objects to hold the information for the simulation
e = Environment()
b = Boat()
b2 = Boat()
b2.hull.set_alpha(0.4)
b2.sail.set_alpha(0.4)

#avatar camera
tilt = 0.0
avhgt = 2.0

# init events
#inputs = pi3d.InputEvents()
#inputs.get_mouse_movement()
# Fetch key presses
mykeys = pi3d.Keyboard()

nextTm = e.tm + DT
i_fr = 0 #water surface image index
wox, woz = 0.0, 0.0 #water uv offset

def restart(e, b):
  b.xm, b.zm, b.heading, b.speed = 0.0, 0.0, 40.0, 2.0
  e.startTm = time.time()
  e.nextPos = e.startTm
  e.posFile.close()
  if WRITEFILE:
    e.posFile = open("models/posFile.txt", "w")
  else:
    e.posFile = open("models/posFile.txt", "r")
  e.compx, e.compz, e.comph, e.comps = 0.0, 0.0, 0.0, 0.5
  e.prevx, e.prevz, e.prevh, e.prevs = 0.0, 0.0, 0.0, 0.5

# Display scene
while DISPLAY.loop_running():
  lastTm = e.tm
  e.tm = time.time()
  frameTm = e.tm - lastTm
  
  # velocity vectors #############################
  dx = -sin(radians(b.heading))
  dz = cos(radians(b.heading))
  
  # move camera ##################################
  xoff = -dx * 4.0
  zoff = -dz * 4.0
  CAMERA.reset()
  CAMERA.rotate(tilt, b.heading, 0)
  CAMERA.position((b.xm + xoff, b.ym + avhgt, b.zm + zoff))
  
  # environment and fog ##########################
  myecube.position(b.xm, b.ym, b.zm)
  myecube.draw()
  
  # draw water ###################################
  wox = (wox + dx * b.speed * frameTm * DWO) % 1.0
  woz = (woz - dz * b.speed * frameTm * DWO) % 1.0
  water.set_offset((wox, woz))
  water.position(b.xm, b.ym, b.zm)
  water.draw()
      
  # draw hull ####################################
  absheel = degrees(asin(sin(radians(b.heel)) * cos(radians(b.heading))))
  abspitch = degrees(asin(-sin(radians(b.heel)) * sin(radians(b.heading))))
  b.hull.position(b.xm, b.ym, b.zm)
  b.hull.rotateToX(abspitch)
  b.hull.rotateToY(-b.heading)
  b.hull.rotateToZ(absheel)
  b.hull.draw()
  # draw sail ####################################
  b.sail.position(b.xm, b.ym, b.zm)
  b.sail.rotateToX(abspitch)
  b.sail.rotateToY(-b.heading - b.sheet)
  b.sail.rotateToZ(absheel)
  b.sail.draw()
  
  # other boat hull ##############################
  b2posx = e.compx + (e.compx - e.prevx) * (2.0 - e.nextPos + e.tm) / 2.0
  b2posz = e.compz + (e.compz - e.prevz) * (2.0 - e.nextPos + e.tm) / 2.0
  b2.hull.position(b2posx, 0.0, b2posz)
  b2.hull.rotateToY(-e.comph)
  b2.hull.draw()
  # other boat sail ##############################
  b2.sail.position(b2posx, 0.0, b2posz)
  b2.sail.rotateToY(-e.comph - e.comps)
  b2.sail.draw()
  
  # check time and animate #######################
  if e.tm > e.nextUp:
    b.updateVariables(e, burgee, burgee2, tiller)
    e.nextUp = e.tm + e.dTm
  if e.tm > nextTm:
    i_fr = (i_fr + 1) % nImg
    water.buf[0].textures[0] = wimg[i_fr]
    nextTm = e.tm + DT
  b.heading += b.wt * frameTm
  b.xm += dx * b.speed * frameTm
  b.zm += dz * b.speed * frameTm

  #inputs.do_input_events()
  #mx, my, mv, mh, md = inputs.get_mouse_movement()
  ########## temp do this
  #heading -= (mx)*0.2
  #tilt -= (my)*0.2
  #if mx > 0.0:
  #  b.turnRight(mx/1000.0)
  #elif mx < 0.0:
  #  b.turnLeft(-mx/1000.0)
  mark1.draw()
  mark2.draw()
  mark3.draw()
  
  tiller.rotateToZ(degrees(b.rudder))
  tiller.draw()
  burgee.draw()
  burgee2.draw()
  mk1.draw()
  mk2.draw()
  mk3.draw()
  boat.position(dw/2.0 - 150.0 + b.xm/2.0, -dh/2.0 + 25.0 + b.zm/2.0, 0.1)
  boat.rotateToZ(b.heading)
  boat.draw()
  boat1.position(dw/2.0 - 150.0 + e.compx/2.0, -dh/2.0 + 25.0 + e.compz/2.0, 0.1)
  boat1.rotateToZ(e.comph)
  boat1.draw()
  
  if e.tm > e.nextPos and not(e.posFile.closed):
    e.nextPos += POSSTEP
    if WRITEFILE:
      # write to file
      e.posFile.write(str(b.xm)+","+str(b.zm)+","+str(b.heading)+
          ","+str(b.heel)+","+str(b.sheet)+"\n")
    else:
      # read from file
      try:
        e.prevx = e.compx
        e.prevz = e.compz
        strArr = e.posFile.readline().split(",")
        e.compx = float(strArr[0])
        e.compz = float(strArr[1])
        e.comph = float(strArr[2])
        e.comps = float(strArr[4])
      except Exception as err:
        print("error:", err)
        e.posFile.close()

  k = mykeys.read()
  if k >-1:
    if (k == 27): #esc
      break
    elif k == 119:  # w
      b.letOut()
    elif k == 115: # s
      b.takeIn()
    #elif inputs.key_state("KEY_A"):
    #  b.turnLeft()
    #elif inputs.key_state("KEY_D"):
    #  b.turnRight()
      
    #elif inputs.key_state("BTN_TOP2"):
    #  b.takeIn()
    #elif inputs.key_state("BTN_BASE"):
    #  b.letOut()
      
    elif k == 120: # x
      restart(e, b)
  
  if R_POT > RPOTMAX:
    RPOTMAX = R_POT
  if R_POT < RPOTMIN:
    RPOTMIN = R_POT
  b.rudder = - RUDDERMIN - (R_POT - RPOTMIN) / (RPOTMAX - RPOTMIN + 0.001) * (RUDDERMAX - RUDDERMIN)
  #"""    
  #right joystick for the tiller  
  #jx, jy = inputs.get_joystickR()
  #b.rudder = -RUDDERMAX * jx * b.rFactor
  #if abs(jx) > 0.02:
  #  if b.rFactor < 0.7:
  #    b.rFactor *= 1.010
  #else:
  #  b.rFactor = 0.2
 
  #left joystick x for hiking, y for sheet
  #jx, jy = inputs.get_joystick()
  #if jy < SPOTMIN:
  #  SPOTMIN = jy
  #if jy > SPOTMAX:
  #  SPOTMAX = jy
  #if jx < HIKEMIN:
  #  HIKEMIN = jx
  #if jx > HIKEMAX:
  #  HIKEMAX = jx
  #newsPot = 1.0 - 2.0 * (jy - SPOTMIN) / (SPOTMAX - SPOTMIN + 0.001)
  #b.sPot = b.sPot * 0.75 + 0.25 * newsPot * SPOTFACTOR
  #b.hike = -1.0 + 2.0 * (jx - HIKEMIN) / (HIKEMAX - HIKEMIN + 0.001)
  
  #if not (jy == 0.0):
  #  tilt = jy * 0.2
  #"""
e.posFile.close()
DISPLAY.destroy()
