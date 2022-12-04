import numpy as np
import matplotlib.pyplot as plt


with open('example_map.txt') as f:
     for line in f:
         if (line[0]=="#"): 
             x_size,y_size = line[1:].split(",")
             x_size,y_size = int(x_size),int(y_size)
             break
        
map =-1*np.ones((x_size,y_size))
x_ = []
y_ = []
with open('example_map.txt') as f:
     for line in f:
         if (line[0]=="#"): continue
         x, y,z = line.split(",")
         x, y,z = int(x), int(y),int(z)
         x_.append(x)
         y_.append(y)
         map[x,y] = z
print(np.unique(map))



# # For 2D
# with open('example.txt') as f:
#      for line in f:
#         #  if (line[0]=="#"): continue
#          x, y = line.split(",")
#          x, y = int(x), int(y)
#          map[x,y] = 150
# # print(x_size,y_size)      
#For 3D
dx = [-1, -1, -1,  0,  0,  1, 1, 1]
dy = [-1,  0,  1, -1,  1, -1, 0, 1]
with open('example_3d.txt') as f:
    
    for line in f:
        x, y, theta = line.split(",")
        x, y, theta = int(x), int(y), int(theta)
        
        
        for i in range(len(dx)):
            # map[x+dx[i],y+dy[i]] = 80
            map[x+2*dx[i],y+dy[i]] = 80
            map[x+dx[i],y+2*dy[i]] = 80
            map[x+3*dx[i],y+dy[i]] = 80
            map[x+dx[i],y+3*dy[i]] = 80
        map[x,y] = 150
with open('example_3d.txt') as f:
    
    for line in f:
        x, y, theta = line.split(",")
        x, y, theta = int(x), int(y), int(theta)
        map[x,y] = 150
            
nrows, ncols = x_size,y_size
# image = np.zeros(nrows*ncols)
image = map
# Set every other cell to a random number (this would be your data)
# image[::2] = np.random.random(nrows*ncols //2 + 1)

# Reshape things into a 9x9 grid.
image = image.reshape((nrows, ncols))

# row_labels = range(nrows)
# col_labels = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I']
plt.matshow(image)
# plt.xticks(range(ncols), col_labels)
# plt.yticks(range(nrows), row_labels)
plt.show()
    
    