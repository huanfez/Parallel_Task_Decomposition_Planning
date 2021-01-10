# Author: Rodolfo Ferro 
# Mail: ferro@cimat.mx
# Script: Compute the Convex Hull of a set of points using the Graham Scan

import sys
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# Function to know if we have a CCW turn
def CCW(p1, p2, p3):
	if (p3[1]-p1[1])*(p2[0]-p1[0]) >= (p2[1]-p1[1])*(p3[0]-p1[0]):
		return True
	return False

# Main function:
def GiftWrapping(S):
	n = len(S)
	P = [None] * n
	l = np.where(S[:,0] == np.min(S[:,0]))
	pointOnHull = S[l[0][0]]
	i = 0
	while True:
		# print 'i', i, 'hull', pointOnHull
		try:
			P[i] = pointOnHull
		except:
			break
			main(S)
		endpoint = S[0]
		for j in range(1,n):
			if (endpoint[0] == pointOnHull[0] and endpoint[1] == pointOnHull[1]) or not CCW(S[j],P[i],endpoint):
				endpoint = S[j]
		i = i + 1
		pointOnHull = endpoint
		if endpoint[0] == P[0][0] and endpoint[1] == P[0][1]:
			break
	for i in range(n):
		if not isinstance(P[-1], np.ndarray):
			if P[-1] == None:
				del P[-1]
	return np.array(P)

def main(P):
	L = GiftWrapping(P)
	# print 'Convex Hull Boundary Points', L, 'number of sides', len(L)
	# Plot the computed Convex Hull:
	# plt.figure(2)
	# plt.plot(L[:,0],L[:,1], 'b-', picker=5)
	# plt.plot([L[-1,0],L[0,0]],[L[-1,1],L[0,1]], 'b-', picker=5)
	# plt.plot(P[:,0],P[:,1],".r")
	# plt.grid()
	# plt.axis('on')
	# plt.show()
	return L
if __name__ == '__main__':
  main(P)
