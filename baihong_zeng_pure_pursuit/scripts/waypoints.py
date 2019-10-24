import csv
import numpy as np
import math

def writeLine(x, y, xx, yy, n):
    x_lst = np.linspace(x, xx, n)
    y_lst = np.linspace(y, yy, n)
    for i in range(n-1):
        writer.writerow([x_lst[i], y_lst[i]])


def writeCorner(x, y, l, r, n, id):
    if id == 1:    
        for i in range(n):
            alpha = theta/n*i
            xx = x + r*math.sin(alpha) - i*0.12
            yy = y + l - r*math.cos(alpha)
            writer.writerow([xx,yy]) 
    elif id == 2:
        for i in range(n):
            alpha = theta/n*i
            xx = x - l + r*math.cos(alpha)
            yy = y + r*math.sin(alpha) - i*0.12
            writer.writerow([xx,yy]) 
    elif id == 3:
        for i in range(n):
            alpha = theta/n*i
            xx = x - r*math.sin(alpha) + i*0.12
            yy = y - l + r*math.cos(alpha)
            writer.writerow([xx,yy])
    elif id == 4:
        for i in range(n):
            alpha = theta/n*i
            xx = x + l - r*math.cos(alpha)
            yy = y - r*math.sin(alpha) + i*0.13
            writer.writerow([xx,yy])

writer = csv.writer(open("waypoints.csv","w"))
n_y = 40 # number of point in the long side
n_x = 10 # number of point in the short side
n_c = 3 # number of point in the corner
l = 1.5 # turn center offset
r = 1 # turn radius

theta = math.pi/2 # left turn angle

x1, y1 = 0.8, 0.05 #-12.3, -0.15
x2, y2 = 21.6, -0.6 #8.4, -0.15

x3, y3 = 22.4, 1.3 #9.7, 1.15
x4, y4 = 23.0, 7.5 #9.7, 7.3

x5, y5 = 21.2, 8.1 #8.4, 8.6
x6, y6 = 0.5, 8.9 #-12.3, 8.6

x7, y7 = -0.3, 7.1 #-13.6, 7.3
x8, y8 = -0.8, 0.7 #-13.6, 1.15 

writeLine(x1, y1, x2, y2, n_y)
writeCorner(x2, y2, l, r, n_c, 1)

writeLine(x3, y3, x4, y4, n_x)
writeCorner(x4, y4, l, r, n_c, 2)

writeLine(x5, y5, x6, y6, n_y)
writeCorner(x6, y6, l, r, n_c, 3)

writeLine(x7, y7, x8, y8, n_x)
writeCorner(x8, y8, l, r, n_c, 4)


