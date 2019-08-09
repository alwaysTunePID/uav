import numpy as np
import matplotlib.pyplot as plt

data_label = input("data:")
nbr_of_plots = data_label.split(" ")
print(nbr_of_plots)
file_names = []
ts = 1/200
for data_file in nbr_of_plots:
    file_names.append(f"controller_{data_file}.log")


font = {'family': 'serif',
        'color':  'darkred',
        'weight': 'normal',
        'size': 16,
        }

data_labels = {
        'imu_data': ['roll angle', 'pitch angle', 'yaw angle',
                'roll rate', 'pitch rate', 'yaw rate'],
        'motor': ['v1', 'v2', 'v3', 'v4'],
        'errors': ['roll angle error', 'pitch angle error',
                'roll rate error', 'pitch rate error', 'yaw rate error'],
        'PID': ['k roll angle', 'k pitch angle',
            'roll rate pid', 'pitch rate pid', 'yaw rate pid','Integral r','Integral p'],
        'references': ['roll rate ref', 'pitch rate ref', 'yaw rate ref'],
    }

# Open file_name safely and store data as a matrix
file_nbr = 0
for file_name in file_names:
    with open(file_name) as file:
        data  = np.loadtxt(file, delimiter=' ')
    length = data.shape[0]
    time = np.transpose(ts*np.arange(0, length))
    counter = 0
    loop_length = int(data.shape[1])
    print(loop_length)
    for i in range(loop_length):
        plt.figure()
        plt.plot(time, data[:,counter],'b')
        plt.title(data_labels[nbr_of_plots[file_nbr]][counter], fontdict=font)
        counter +=1

    file_nbr +=1

# This should be changed



#print(time)
#print(data)



#print(loop_length)
# labels = ['u1 v1','u2 v2',
#         'u3 v3','u4 v4',
#         'pitch roll', 'Ppitch Proll',
#         'I_a_p I_a_r', 'K-del rate loop'
#         ]



#     plt.plot(time, data[:,counter],'orange')
#     counter+=1
    

plt.show()




