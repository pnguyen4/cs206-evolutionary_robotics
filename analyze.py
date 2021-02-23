import matplotlib.pyplot as plt
import numpy as np

backLegSensorValues = np.load('data/back_leg_sensor_values.npy')
frontLegSensorValues = np.load('data/front_leg_sensor_values.npy')
plt.plot(backLegSensorValues, label='back leg', linewidth=4)
plt.plot(frontLegSensorValues, label='front leg')
plt.legend()
plt.show()
