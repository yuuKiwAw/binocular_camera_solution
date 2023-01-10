import matplotlib.pyplot as plt
from plyfile import *
from mpl_toolkits.mplot3d import Axes3D

plydata = PlyData.read(r'D:\source\python_project\dualcamera\bin\Release\pointCloud.ply')

xlist = plydata['vertex']['x']
ylist = plydata['vertex']['y']
zlist = plydata['vertex']['z']

fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(xlist, ylist, zlist)
plt.show()
