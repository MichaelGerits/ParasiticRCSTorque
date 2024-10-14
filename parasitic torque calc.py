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
        self.parasiticTorque = self.actualTorque - self.actualTorque

        if parasitic == True:
            return (self.actualTorque, self.forceScale * self.forceUnitNew)
        else:
            return (self.intendedTorque, self.forceScale * self.forceUnit)

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

'''
Here all ofthe thrusters are defined
'''
T1 = Thruster(
    position= [-1, -1, 1],
    point= [-1, 0, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T2 = Thruster(
    position= [-1, 1, 1],
    point= [-1, 0, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T3 = Thruster(
    position= [0, 2, 1],
    point= [0, 1, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T4 = Thruster(
    position= [1, 1, 1],
    point= [1, 0, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T5 = Thruster(
    position= [1, -1, 1],
    point= [1, 0, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T6 = Thruster(
    position= [0, -2, 1],
    point= [0, -1, 0],
    misalignment= [0,0,0],
    force= 2.2
)

T7 = Thruster(
    position= [-1, -1, -1],
    point= [-1, 0, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T8 = Thruster(
    position= [-1, 1, -1],
    point= [-1, 0, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T9 = Thruster(
    position= [0, 2, -1],
    point= [0, 1, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T10 = Thruster(
    position= [1, 1, -1],
    point= [1, 0, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T11 = Thruster(
    position= [1, -1, -1],
    point= [1, 0, 0],
    misalignment= [0,0,0],
    force= 2.2
)
T12 = Thruster(
    position= [0, -2, -1],
    point= [0, -1, 0],
    misalignment= [0,0,0],
    force= 2.2
)

'''
Here all of the maneuvres are defined
'''
xmove = Maneuvre([T1, T2, T7, T8])
xmoveNeg = Maneuvre([T4, T5, T10, T11])

ymove = Maneuvre([T6, T12])
ymoveNeg = Maneuvre([T3, T9])

xrot = Maneuvre([T3, T12])
xrotNeg = Maneuvre([T6, T9])

yrot = Maneuvre([T1, T2, T10, T11])
yrotNeg = Maneuvre([T5, T4, T7, T8])

zrot = Maneuvre([T1, T4, T7, T10])
zrotNeg = Maneuvre([T2, T5, T8, T11])

plotThrusters(active=[T1, T2, T3, T4, T5, T6] ,inactive=[T7, T8, T9, T10, T11, T12])