import matplotlib.pyplot as plt 
import numpy as np 

f = open('stats_brrt.txt','r')
f = f.readlines()
nodes = []
nodes_a = []
times = []
times_a = []
lengths = []
lengths_a = []

ns,ts,ls = 0,0,0
t = 0
for i in f:
	line = i.split('|')
	# print type(float(line[0]))
	nodes.append(float(line[0]))
	times.append(float(line[1]))
	lengths.append(line[2])
	ns,ts,ls = ns+float(line[0]),ts+float(line[1]),ls+float(line[2])
print ns,ts,ls
for i in nodes:
	nodes_a.append(ns/10)
	times_a.append(ts/10)
	lengths_a.append(ls/10)

plt.figure(1)
plt.plot(nodes,label="Individual")
plt.plot(nodes_a,label="Average")
plt.legend(loc=0)
plt.title("Number of Nodes Expanded")
plt.savefig("nodes_expanded.jpg")
plt.figure(2)
plt.plot(times,label="Individual")
plt.plot(times_a,label="Average")
plt.legend(loc=0)
plt.title("Time Taken to Find the Solution in Seconds")
plt.savefig("time_taken.jpg")
plt.figure(3)
plt.plot(lengths,label="Individual")
plt.plot(lengths_a,label="Average")
plt.legend(loc=0)
plt.title("Cost of the Solution")
plt.savefig("lengths.jpg")
plt.show()