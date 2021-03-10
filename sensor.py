import numpy as np
import pyrosim.pyrosim as pyrosim


class SENSOR:
    def __init__(self, linkname):
        self.linkName = linkname
        self.values = np.zeros(1000)

    def Get_Value(self, timestep):
        self.values[timestep] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

    def Save_Values(self):
        np.save(f'data/{self.linkName}_values.npy', self.values)
