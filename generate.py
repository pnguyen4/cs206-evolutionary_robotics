import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("box.sdf")

# Link Size
length = 1
width = 1
height = 1

# Link Position
x = 0
y = 0
z = 0.5

# pyrosim.Send_Cube(name="Box", pos=[x, y, z], size=[length, width, height])
# pyrosim.Send_Cube(name="Box2", pos=[x+1, y, z+1], size=[length, width, height])

for i in range(0, 6):
    for j in range (0, 6):
        for k in range(1, 11):
            pyrosim.Send_Cube(name="Box2",
                              pos=[x+i, y+j, z+k],
                              size=[length * (.9**k),
                                    width * (.9**k),
                                    height * (.9**k)])

pyrosim.End()
