import matplotlib.pyplot as plt
import numpy as np

backLegSensorValues = np.load('data/back_leg_sensor_values.npy')
plt.plot(backLegSensorValues)
plt.show()
