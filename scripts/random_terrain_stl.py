#===============================================================================
# Create a random map
#===============================================================================

import sys
import numpy as np
import time
from stl_tools import numpy2stl


from diamond_square import * #@UnusedWildImport


#Init
WHITE = (255,255,255)
BLACK = (0,0,0)
WATER = (0,148,255)
DEEPWATER = (0,0,255)
SAND = (255,178,127)
GRASS = (0,255,0)
FORREST = (0,127,14)
STONE = (64,64,64)

GAME_WIDTH = 512
GAME_HEIGHT = 512

SCALING = 0.1 #The size of the generated map


#coordinates = Sparks(GAME_WIDTH/SCALING,GAME_HEIGHT/SCALING)
#coordinates = ParticleDeposition(GAME_WIDTH/SCALING,GAME_HEIGHT/SCALING)
#coordinates = HillAlgorithm(GAME_WIDTH/SCALING,GAME_HEIGHT/SCALING)
#coordinates = HillAlgorithm_mod(GAME_WIDTH/SCALING,GAME_HEIGHT/SCALING)
#coordinates = ValueNoise2(GAME_WIDTH/SCALING,GAME_HEIGHT/SCALING)
coordinates = DiamondSquare(513) #The size has to be 2^n+1

mapHeights = np.zeros((513,513))
rowCounter = 0
columnCounter = 0
for item in coordinates:
    mapHeights[rowCounter,columnCounter] = item[2];
    columnCounter = columnCounter+1
    if columnCounter > 512:
        columnCounter = 0
        rowCounter = rowCounter+1

mapHeights = mapHeights-mapHeights.min()
mapHeights = mapHeights*SCALING+1
numpy2stl(mapHeights, "test.stl", scale=1, mask_val=0, solid=True)