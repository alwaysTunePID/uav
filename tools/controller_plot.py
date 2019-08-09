import numpy as np
import matplotlib.pyplot as plt

data_label = input("data:")
file_name = f"controller_{data_label}.log"

# Open file_name safely and store data as a matrix
with open(file_name) as file:
    data  = np.loadtxt(file, delimiter=' ')

# This should be changed
ts = 1/200
length = data.shape[0]
time = np.transpose(ts*np.arange(0, length))

#print(time)
#print(data)
font = {'family': 'serif',
        'color':  'darkred',
        'weight': 'normal',
        'size': 16,
        }

counter = 0
loop_length = int(data.shape[1])
print(loop_length)
# labels = ['u1 v1','u2 v2',
#         'u3 v3','u4 v4',
#         'pitch roll', 'Ppitch Proll',
#         'I_a_p I_a_r', 'K-del rate loop'
#         ]
data_labels = {
        'imu_data': ['roll angle', 'pitch angle', 'yaw angle',
                        'roll rate', 'pitch rate', 'yaw rate'],
        'motor': ['v1', 'v2', 'v3', 'v4'],
        'errors': ['roll angle error', 'pitch angle error',
                'roll rate error', 'pitch rate error', 'yaw rate error'],
        'PID': ['k roll angle', 'k pitch angle',
                'roll rate pid', 'pitch rate pid', 'yaw rate pid'],
        'references': ['roll rate ref', 'pitch rate ref', 'yaw rate ref'],
        }

for i in range(loop_length):
    plt.figure()
    plt.plot(time, data[:,counter],'b')
    plt.title(data_labels[data_label][counter], fontdict=font)
    counter +=1
#     plt.plot(time, data[:,counter],'orange')
#     counter+=1
    

plt.show()




