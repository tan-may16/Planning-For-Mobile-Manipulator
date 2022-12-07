import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
parser = argparse.ArgumentParser(description='2D or 3D.')
parser.add_argument('--type', type=str, default='2D')
args = parser.parse_args()

base_path = os.path.dirname(os.path.abspath(__file__))

with open(base_path + '/office_map.txt') as f:
     for line in f:
         if (line[0]=="#"): 
             x_size,y_size = line[1:].split(",")
             x_size,y_size = int(x_size),int(y_size)
             break
        
map =-1*np.ones((x_size,y_size))
x_ = []
y_ = []
with open(base_path +'/office_map.txt') as f:
     for line in f:
         if (line[0]=="#"): continue
         x, y,z = line.split(",")
         x, y,z = int(x), int(y),int(z)
         x_.append(x)
         y_.append(y)
         map[x,y] = z



# For 2D
if (args.type == '2D'):
    with open(base_path +'/office_2d.txt') as f:
        for line in f:
            #  if (line[0]=="#"): continue
            x, y = line.split(",")
            x, y = int(x), int(y)
            map[x,y] = 150
    image = map
    plt.matshow(image.T)
    plt.imsave(base_path+'/plan_2D.png', image.T)
    plt.show()
    



#For 3D
else:
    dx = [-1, -1, -1,  0,  0,  1, 1, 1]
    dy = [-1,  0,  1, -1,  1, -1, 0, 1]
    with open(base_path +'/office_3d.txt') as f:
        for line in f:
            x, y, theta = line.split(",")
            x, y, theta = int(x), int(y), int(theta)
            map[int(x),int(y)] = 150
            
        
    with open(base_path +'/office_3d.txt') as f:
        
        for line in f:
            x, y, theta = line.split(",")
            x, y, theta = int(x), int(y), int(theta)
            map[x,y] = 150
            
    image = map
    plt.matshow(image.T)
    plt.imsave(base_path+'/plan_3D.png', image.T)
    plt.show()
        
    