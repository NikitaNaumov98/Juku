import numpy as np
import matplotlib.pyplot as plt

x = np.array([3312, 3285, 3248, 3159, 3129])
y = np.array([2.45,2.25,2.15,1.9, 1.7])

m,b = np.polyfit(x,y,1)

print(m)
print(b)

plt.scatter(x,y,color="#003F72")
plt.plot(x,y,"o")

plt.plot(x,m*x + b)

plt.show()

def get_speed(y):
    global m,b
    return (y - b)/m 
