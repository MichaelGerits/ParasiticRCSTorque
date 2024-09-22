import numpy as np

class Truster:
    def __init__(self, position, point, misalignment, force):
        '''
        this defines the position and the ponting of an RCS thruster, with an additional offset:
        
        -All vectors are taken for a body fixed axis centered at the centre of mass
        -The angles (alpha, beta, gamma) correspond to sequential rotations around a fixed xyz axis
        '''
        self.position = position
        self.forceUnit = -point/np.linalg.norm(point)
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
    
    def calcTorque(self, parasitic = True):
        self.intendedTorque = np.cross(self.position, self.forceScale * self.forceUnit)
        self.actualTorque = np.cross(self.position, self.forceScale * self.forceUnitNew)
        self.parasiticTorque = self.actualTorque - self.actualTorque