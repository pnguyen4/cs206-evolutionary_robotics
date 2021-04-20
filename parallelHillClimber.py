import copy
import constants as c
import glob
from solution import SOLUTION
import os


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        if glob.glob("brain*.nndf"):
            os.system("rm brain*.nndf")
        if glob.glob("fitness*.txt"):
            os.system("rm fitness*.txt")
        self.parents = {}
        self.nextAvailableID = 0
        for i in range(0, c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
        # self.parent = SOLUTION()

    def Evolve(self):
        self.Evaluate(self.parents)
        # self.parent.Evaluate("GUI")
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()

    def Evaluate(self, solutions):
        for i in range(0, c.populationSize):
            solutions[i].Start_Simulation("DIRECT")
        for i in range(0, c.populationSize):
            solutions[i].Wait_For_Simulation_To_End()

    def Spawn(self):
        self.children = {}
        for p in self.parents:
            self.children[p] = copy.deepcopy(self.parents[p])
            self.children[p].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for child in self.children:
            self.children[child].Mutate()

    def Select(self):
        for p in self.parents:
            if self.children[p].fitness < self.parents[p].fitness:
                self.parents[p] = self.children[p]

    def Print(self):
        print("")
        for p in self.parents:
            output = f"Parent{p} Fitness: {self.parents[p].fitness},"
            output += f" Child Fitness: {self.children[p].fitness}"
            print(output)
        print("")

    def Show_Best(self):
        best = 0
        for p in self.parents:
            if self.parents[p].fitness < self.parents[best].fitness:
                best = p
        self.parents[best].Start_Simulation("GUI")
