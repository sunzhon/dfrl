
import numpy as np
import matplotlib.pyplot as plt
 
filename = 'o1.txt'
data = np.loadtxt(filename, dtype=float)       # 使用的数据类型

plt.plot(data)
plt.show()
print(data)

