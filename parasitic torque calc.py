import numpy as np
import matplotlib.pyplot as plt

class Thruster:
    def __init__(self, position, point, misalignment, force):
        '''
        this defines the position and the ponting of an RCS thruster, with an additional offset:
        
        -All vectors are taken for a body fixed axis centered at the centre of mass and are in SI units
        -The angles (alpha, beta, gamma) correspond to sequential rotations around a fixed xyz axis in that order
        '''
        self.position = np.array(position)
        self.forceUnit = -np.array(point)/np.linalg.norm(np.array(point))
        self.forceScale = force
        self.alpha, self.beta, self.gamma = misalignment

        #https://en.wikipedia.org/wiki/Rotation_matrix (extrinsic rotation) (x-y-z rotation)
        self.xMatrix = np.array([[1, 0                 ,  0                 ],
                                 [0, np.cos(self.alpha), -np.sin(self.alpha)],
                                 [0, np.sin(self.alpha),  np.cos(self.alpha)]])
        
        self.yMatrix = np.array([[np.cos(self.beta) , 0, np.sin(self.beta)],
                                 [0                 , 1, 0                ],
                                 [-np.sin(self.beta), 0, np.cos(self.beta)]])
        
        self.zMatrix = np.array([[np.cos(self.gamma), -np.sin(self.gamma), 0],
                                 [np.sin(self.gamma),  np.cos(self.gamma), 0],
                                 [0                 ,  0                 , 1]])
        
        
        
        self.rotMatrix = np.matmul(self.xMatrix, np.matmul(self.yMatrix, self.zMatrix))

        self.forceUnitNew = np.matmul(self.rotMatrix, self.forceUnit)
    
    def __str__(self):
        return f"pos: {self.position}, point: {self.forceUnit}, missAll.: {[self.alpha, self.beta, self.gamma]}"

    def calcEffect(self, parasitic = True):
        '''
        calculates the force and torque effect due to the thruster
        '''
        self.intendedTorque = np.cross(self.position, self.forceScale * self.forceUnit)
        self.actualTorque = np.cross(self.position, self.forceScale * self.forceUnitNew)
        self.parasiticTorque = self.actualTorque - self.intendedTorque

        if parasitic == True:
            return (self.parasiticTorque, self.forceScale * self.forceUnitNew)
        else:
            return (self.intendedTorque, self.forceScale * self.forceUnit)

def plotThrusters(active = [], inactive = []):
    max_val = 3
    fig = plt.figure( figsize = (10,8) )
    ax  = fig.add_subplot( 111, projection = '3d')

    #pitterates over each thruster
    for thruster in active + inactive:
        position = thruster.position
        point = -thruster.forceUnit 
        #plots the posiiton
        if thruster in active:
            color = 'green'
        else:
            color = 'red'
        ax.plot( [ position[0] ], [ position[1] ], [ position[2] ], 
                'o',
        	    color = color )		
        max_val = max( [ abs(position).max(), max_val ] )

	#--------------------------------------------------------------------------------------------------
	#Thruster pointing direction
        ax.quiver( position[0], position[1], position[2], point[0], point[1], point[2], color = color, lw=2, hatch='O')

	#---	----------------------------------------------------------------------------------------------------------

	#plots the central body axes
    x, y, z = [ [ 0, 0, 0 ], [ 0, 0, 0  ], [ 0, 0, 0 ] ]
    u, v, w = [ [ 2, 0, 0 ], [ 0, 2, 0 ], [ 0, 0, 2 ] ]
    ax.quiver( x, y, z, u, v, w, color = 'b' )
	#adds in the labels
    xlabel = 'X (%s)' % 'm'
    ylabel = 'Y (%s)' % 'm'
    zlabel = 'Z (%s)' % 'm' 
	
    ax.set_xlim( [ -max_val, max_val ] )
    ax.set_ylim( [ -max_val, max_val ] )
    ax.set_zlim( [ -max_val, max_val ] )
    ax.set_xlabel( xlabel )
    ax.set_ylabel( ylabel )
    ax.set_zlabel( zlabel )
    ax.set_box_aspect( [ 1, 1, 1 ] )
    ax.set_aspect( 'auto' )	
    plt.show()

class Maneuvre:
    def __init__(self, thrusters):
        self.thrusters = thrusters
        

    def __str__(self):
        return f"{self.thrusters}"
        
    def preformManeuvre(self, parasitic=True):
        self.T = np.zeros(3)
        self.F = np.zeros(3)
        for thruster in self.thrusters:
            effect = thruster.calcEffect(parasitic)
            self.T += effect[0]
            self.F += effect[1]
        print(f"Force: {self.F}, Torque: {self.T}")
    def plot(self):
        plotThrusters(active=self.thrusters)
'''
Here all of the thrusters are defined
'''
T1 = Thruster(
    position= [-0.01, 0.94, 0.74],
    point= [0, 1, 0],
    misalignment= [0,0,0],
    force= 10
)
T2 = Thruster(
    position= [-0.88, 0.45, 0.74],
    point= [-1, 0, 0],
    misalignment= [0,0,0],
    force= 10
)
T3 = Thruster(
    position= [-0.88, -0.45, 0.74],
    point= [-1, 0, 0],
    misalignment= [0,0,0],
    force= 10
)
T4 = Thruster(
    position= [-0.01, -0.99, 0.74],
    point= [0, -1, 0],
    misalignment= [0,0,0],
    force= 10
)
T5 = Thruster(
    position= [0.88, -0.45, 0.74],
    point= [1, 0, 0],
    misalignment= [0,0,0],
    force= 10
)
T6 = Thruster(
    position= [0.88, 0.45, 0.74],
    point= [1, 0, 0],
    misalignment= [0,0,0],
    force= 10
)

T7 = Thruster(
    position= [-0.01, 0.94, -0.72],
    point= [0, 1, 0],
    misalignment= [0,0,0],
    force= 10
)
T8 = Thruster(
    position= [-0.88, 0.45, -0.72],
    point= [-1, 0, 0],
    misalignment= [0,0,0],
    force= 10
)
T9 = Thruster(
    position= [-0.88, -0.45, -0.72],
    point= [-1, 0, 0],
    misalignment= [0,0,0],
    force= 10
)
T10 = Thruster(
    position= [-0.01, -0.99, -0.72],
    point= [0, -1, 0],
    misalignment= [0,0,0],
    force= 10
)
T11 = Thruster(
    position= [0.88, -0.45, -0.72],
    point= [1, 0, 0],
    misalignment= [0,0,0],
    force= 10
)
T12 = Thruster(
    position= [0.88, 0.45, -0.72],
    point= [1, 0, 0],
    misalignment= [0,0,0],
    force= 10
)

'''
Here all of the maneuvres are defined
'''
xmove = Maneuvre([T2, T3, T8, T9])
xmoveNeg = Maneuvre([T5, T6, T11, T12])

ymove = Maneuvre([T4, T10])
ymoveNeg = Maneuvre([T1, T7])

xrot = Maneuvre([T10, T1])
xrotNeg = Maneuvre([T4, T7])

yrot = Maneuvre([T11, T12, T2, T3])
yrotNeg = Maneuvre([T5, T6, T8, T9])

zrot = Maneuvre([T3, T6, T9, T12])
zrotNeg = Maneuvre([T2, T5, T8, T11])

#plotThrusters(active=[T1, T2, T3, T4, T5, T6] ,inactive=[T7, T8, T9, T10, T11, T12])

ymove.preformManeuvre(parasitic=False)
ymove.plot()
