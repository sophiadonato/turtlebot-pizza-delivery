import numpy as np
import matplotlib.pyplot as plt

A = np.zeros((100,100))

def isSeenByCamera(i,j, robot, URcorner, ULcorner):
    # Return false if (i,j) is not in the camera field of vision, true if yes
    distance = np.sqrt(10**2 + 5**2)
    if(EuclideanDistance(i,j, robot[0], robot[1]) > distance): #Which value to consider => value in map discretization (not in cm !)
        return False
    if(EuclideanDistance(i,j, URcorner[0],URcorner[1]) > distance):
        return False
    if(EuclideanDistance(i,j, ULcorner[0], ULcorner[1]) > distance):
       return False
    return True

def EuclideanDistance(i1,j1, i2,j2):
    return np.sqrt((i1-i2)**2 + (j1-j2)**2)

robot = (50,60)
ULcorner = (60,55)
URcorner = (60,65)

for i in range(100):
    for j in range(100):
        if(isSeenByCamera(i,j, robot, URcorner,ULcorner)):
                A[i][j] = 1
A[50][60] = 2
A[60][55] = 2
A[60][65] = 2

# Function to show the heat map
plt.imshow( A , cmap = 'magma' )
  
# Adding details to the plot
plt.title( "2-D Heat Map" )
plt.xlabel('x-axis')
plt.ylabel('y-axis')

# Adding a color bar to the plot
plt.colorbar()

# Displaying the plot
plt.savefig("ProveOfConcept.pdf")
plt.show()