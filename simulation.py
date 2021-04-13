import constants as c
import pybullet as p
import pybullet_data
import time
from world import WORLD
from robot import ROBOT


class SIMULATION:
    def __init__(self, directOrGui, solutionID):
        if directOrGui == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        elif directOrGui == "GUI":
            self.physicsClient = p.connect(p.GUI)
        else:
            print("Invalid arg: ", directOrGui)
            exit()
        self.directOrGui = directOrGui
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.world = WORLD()
        self.robot = ROBOT(solutionID)

    def __del__(self):
        p.disconnect()

    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def Run(self):
        for i in range(0, c.numTimeSteps):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)
            if self.directOrGui == "GUI":
                time.sleep(1/60)
