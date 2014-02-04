#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# code originally came from a forum:http://gamedev.stackexchange.com/questions/23625/how-do-you-generate-tileable-perlin-noise
# suggestion by boojum. I have added an extra dimension for time continuity

import random
import math

class Noise3D():
 # initialize class with the grid size (inSize), frequency (inFreq) and number of octaves (octs) 
    def __init__(self, inSize, inFreq, octs, seedVal=1):
        self.perm = [i for i in range(256)]
        random.seed(seedVal)
        random.shuffle(self.perm)
        self.perm += self.perm
        self.dirs = [(math.cos(a * 2.0 * math.pi / 256),
                 math.cos((a+85) * 2.0 * math.pi / 256),
                 math.cos((a+170) * 2.0 * math.pi / 256))
                 for a in range(256)]
        self.size = inSize
        self.freq = inFreq
        self.octs = octs
        

    def noise(self, x, y, z, per):
        def surflet(gridX, gridY, gridZ):
            distX, distY, distZ = abs(x-gridX), abs(y-gridY), abs(z-gridZ)
            polyX = 1 - 6*distX**5 + 15*distX**4 - 10*distX**3
            polyY = 1 - 6*distY**5 + 15*distY**4 - 10*distY**3
            polyZ = 1 - 6*distZ**5 + 15*distZ**4 - 10*distZ**3
            hashed = self.perm[self.perm[self.perm[int(gridX)%per] + int(gridY)%per] + int(gridZ)%per]
            grad = (x-gridX)*self.dirs[hashed][0] + (y-gridY)*self.dirs[hashed][1] + (z-gridZ)*self.dirs[hashed][2]
            return polyX * polyY * polyZ * grad
            
        intX, intY, intZ = int(x), int(y), int(z)
        return (surflet(intX+0, intY+0, intZ+0) + surflet(intX+0, intY+0, intZ+1) + surflet(intX+0, intY+1, intZ+0) +
                surflet(intX+0, intY+1, intZ+1) + surflet(intX+1, intY+0, intZ+0) + surflet(intX+1, intY+0, intZ+1) +
                surflet(intX+1, intY+1, intZ+0) + surflet(intX+1, intY+1, intZ+1))
 
    #return a value for noise in 3D volume (2D over time etc)
    def generate(self, x, y, z):
        val = 0
        x = x * self.freq
        y = y * self.freq
        z = z * self.freq
        per = int(self.freq * self.size)
        for o in range(self.octs):
            val += 0.5**o * self.noise(x*2**o, y*2**o, z*2**o, per*2**o)
        return val
    
"""
# this is some code to use the Noise3D to create a water surface effect i.e. has transparency to allow the dark to show through
# in the foreground. Made into a simple animated tile with Panda3D. 
# Other (not commented out version!) is artificial colour for visualizing noise patterns

from PIL import Image

size = 128
freq = 1/32.0
octs = 5
    
nObj = Noise3D(size, freq, octs, 6)

#rCurve = [0, 2, 4, 150, 12, 16, 20, 30, 40, 50, 60, 70, 90, 120, 180, 250, 255]
#gCurve = [50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 170, 200, 220, 245, 250, 250, 255]
#bCurve = [10, 20, 30, 40, 50, 60, 70, 80, 90, 110, 130, 190, 200, 253, 254, 255, 255]
#aCurve = [50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 170, 200, 220, 245, 250, 250, 255]
rCurve = [0, 0, 0, 0, 0, 0, 0, 50, 100, 150, 200, 250, 255, 200, 150, 100, 50]
gCurve = [0, 0, 100, 150, 200, 250, 255, 200, 150, 100, 50, 0, 0, 0, 0, 0, 0 ]
bCurve = [0, 20, 255, 200, 150, 125, 100, 80, 60, 40, 20, 0, 0, 0, 0, 0, 0]
aCurve = [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]

for z in range (min(10,size)):
    data = []  
    print z  
    for y in range(size):
        for x in range(size):
            fBmVal = nObj.generate(x, y, z)
            iV = int(fBmVal*16) % 16
            
            data.append((int(rCurve[iV] + (rCurve[iV+1] - rCurve[iV]) * (fBmVal - iV/16)), 
                    int(gCurve[iV] + (gCurve[iV+1] - gCurve[iV]) * (fBmVal - iV/16)),
                    int(bCurve[iV] + (bCurve[iV+1] - bCurve[iV]) * (fBmVal - iV/16)),
                    int(aCurve[iV] + (aCurve[iV+1] - aCurve[iV]) * (fBmVal - iV/16))))
        
    im = Image.new("RGBA", (size, size))
    im.putdata(data, 128.0, 128.0)
    im.save("classified"+format(z, '03d')+".png")

print "finished doing it"
"""
