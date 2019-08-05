import numpy as np
import matplotlib.pyplot as plt

file_name = "controller.log"

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
loop_length = int(data.shape[1]/2)
print(loop_length)
labels = ['u1 v1','u2 v2',
        'u3 v3','u4 v4',
        'pitch roll', 'Ppitch Proll',
        'I_a_p I_a_r', 'K-del rate loop'
        ]
for i in range(loop_length):
    plt.figure()
    plt.plot(time, data[:,counter],'b')
    counter +=1
    plt.plot(time, data[:,counter],'orange')
    counter+=1
    plt.title(labels[i], fontdict=font)

plt.show()




