import numpy as np
import matplotlib.pyplot as plt

DEV = False
MAP_SIZE = (40,40)
MAP_FILE = '../data/map_c5.txt'

# MAP_BLOCK = [[10,5,30,20],[60,10,90,20],[27,35,50,60],[80,35,85,42],[7,80,17,88],[65,60,92,90]]
# MAP_BLOCK = [[20,20,80,80]]
MAP_BLOCK = [[10*i+2,10*j+2,10*i+8,10*j+8] for i in range(10) for j in range(10)] # for man

#MAP_BLOCK = []

mapdata = np.zeros(MAP_SIZE)

for block in MAP_BLOCK:
	mapdata[block[0]:block[2],block[1]:block[3]]=1

plt.imshow(mapdata)
plt.show()

# if DEV:
# 	mapdata = mapdata*499+1
np.savetxt(MAP_FILE,mapdata,'%d')