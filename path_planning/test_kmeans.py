from sklearn.cluster import KMeans
import numpy as np
import random 
import matplotlib.pyplot as plt 
import pdb
import scipy.linalg as la

x = []
y = []

rad = []
area=[]
obs_cords=[]

numobs = 55

for i in range(numobs):
  pt = (random.randint(0,1000))
  r = (random.randint(-250,250))
  x.append(pt)
  y.append(pt+r)
  rad.append(random.randint(5,25))
  area.append(rad[i]**2)

xcord = []
ycord = []

for i in range(numobs):
  for j in range(rad[i]):
    for k in range(rad[i]):
      xcord.append( x[i]-rad[i]+ 2*j )
      ycord.append( y[i]-rad[i]+ 2*k )
      obs_cords.append(np.array([x[i]-rad[i]+2*j , y[i]-rad[i]+2*k]))
# print(len(obs_cords))

kmeans = KMeans(n_clusters=20, random_state=100).fit_predict(obs_cords)

plt.scatter(xcord, ycord, c = kmeans,lw = 0)

# print(kmeans)

# plot = ax.scatter([], [], s=4, facecolors='r', edgecolors='r')
# ax.set_xlim(0, 1000)
# ax.set_ylim(0, 1000)
# array = list(zip(xcord, ycord))
# plot.set_offsets(array)
# # plt.show()
# fig.canvas.draw()
# plt.scatter(xcord, ycord, s=4, facecolors='r', edgecolors='r')
plt.show()

# kmeans.predict([[0, 0], [4, 4]])
# kmeans.cluster_centers_
