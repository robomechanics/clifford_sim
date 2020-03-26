#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import random
from generateTerrain import GenerateSTL
import numpy as np

GenerateSTL(513,0.1,"../models/meshes/roughTerrain.stl");