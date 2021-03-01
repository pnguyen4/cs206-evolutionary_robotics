import matplotlib.pyplot as plt
import numpy as np

# backLegSensorValues = np.load('data/back_leg_sensor_values.npy')
# frontLegSensorValues = np.load('data/front_leg_sensor_values.npy')
# plt.plot(backLegSensorValues, label='back leg', linewidth=4)
# plt.plot(frontLegSensorValues, label='front leg')
# plt.legend()
# plt.show()

targetAngles_fleg = np.load('data/targetAngles_fleg.npy')
targetAngles_bleg = np.load('data/targetAngles_bleg.npy')
x = np.linspace(-np.pi, np.pi, 1000)
plt.plot(x, targetAngles_fleg)
plt.plot(x, targetAngles_bleg)
plt.show()
