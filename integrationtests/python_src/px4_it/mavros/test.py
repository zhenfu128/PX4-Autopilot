#!/usr/bin/env python2

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt

fig=plt.figure()
# ax=fig.add_subplot(111,projection='3d')

# for rotate the axes and update.
plt.axis('off')
plt.axis('equal')
offset = 20
plt.xlim([20 - offset, 80 + offset])
plt.ylim([40 - offset, 100 + offset])
plt.plot(50,50)
plt.show()
