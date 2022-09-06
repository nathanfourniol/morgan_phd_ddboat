import numpy as np
import matplotlib.pyplot as plt


# Log file
path = "./Log/"
filename = "DDBOAT8_t_2022_9_2_0_0.log"
X = []
Y = []
c = -1
with open(path+filename, "r") as f:
    for line in f:
        line = line.split("/")

        for i in range(len(line)):
            if "KAL" in line[i]:
                line = line[i].split()
                X.append(float(line[2][2:len(line[2])-2]))
                Y.append(float(line[3][1:len(line[3])-2]))
            if
                
X = np.array(X)
Y = np.array(Y)

ax = plt.subplot()
ax.axis('equal')
plt.xlim([-50, 50])
plt.ylim([-50, 50])

for i in range(len(X)):
    # plt.cla()
    ax.plot(X[i-1:i+1], Y[i-1:i+1], '-b')
    plt.draw()
    plt.pause(0.01)


