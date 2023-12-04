from matplotlib import pyplot as plt
plt.rcParams["figure.figsize"] = [7.5, 3.5]
plt.rcParams["figure.autolayout"] = True
fig=plt.figure()
ax = fig.add_subplot(projection="3d")
x, y, z = [1, 1.5], [1, 2.4], [3.4, 1.4]
ax.scatter(x, y, z, c="red")
ax.plot(x, y, z, color="black")
plt.show()