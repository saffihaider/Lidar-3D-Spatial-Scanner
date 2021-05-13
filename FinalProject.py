#Saffi Haider, 400255941, haides23
#Python version 3.8.0

#Import libraries
import serial
import open3d as o3d
import numpy as np
import pyrender
import trimesh
import math as m

 
s = serial.Serial("COM6", 115200) #Open serial line
f = open("tof_radar.xyz", "w") #Open file in write mode

#Initialize counter and x distance
x = 0.00
count = 0


while (count != 10): #Iterate through 10 measurements
    if(s.read().decode() == '*'): #Wait for "start measurement" from microcontroller
        print("Scanning plane: ")
        for i in range(256): #Iterate through all 256 measurements
            datastr = s.readline().decode() #Read and decode data package
            datalist = datastr.split(',') #Seperate package by comma
            distance = int(datalist[0]) #Store distance as int
            angle = m.radians(float(datalist[1])) #Store angle as float and convert to radians

            #Convert polar coordinates to cartesian
            y = distance*m.cos(angle) 
            z = distance*m.sin(angle)

            outputstr = (str(x) + " " + str(y) + " " + str(z) + "\n") #Concatenate XYZ string
            print(str(angle)) #Print angle to screen for user
            print(outputstr) #Print data to screen for user
            f.write(outputstr) #Write to XYZ file
        x += 200; #Increment X for each step in X directon
        count += 1 #Increment count

print("Closing:")
f.close() #Close file
s.close() #Close serial line

pcd = o3d.io.read_point_cloud("tof_radar.xyz", format='xyz') #Create point cloud from XYZ file
numpoints = len(pcd.points) #Get number of points in point cloud
lines = []
i = 0
while (i < (numpoints/512)): #Iterate through point cloud
    lines.append([0+(512*i),511+(512*i)]) #Join each point in cloud
    for x in range(511):
        lines.append([x+(i*512),x+1+(i*512)]) #Join with each other point in cloud
    i += 1 #Increment i
i = 1 #Reset i

#Create line set from connected point cloud
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set]) #Visualize final line set
