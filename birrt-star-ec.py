#!/usr/bin/env python

# birrt-star-ec.py
# This program generates a 
# asymptotically optimal rapidly exploring random tree (RRT* proposed by Sertac Keraman, MIT) in a rectangular region.
#
# RRT* code is taken from https://www.linkedin.com/pulse/motion-planning-algorithm-rrt-star-python-code-md-mahbubur-rahman/
# and modified to implement Bi-directional RRT*#
# 

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
from lineIntersect import *
import copy
import pdb
import time


#constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 2000
RADIUS=15
# OBS=[(50,0,50,300), (200,100,50,400), (350,0,50,300), (500,100,50,400)]
OBS=[(500,150,100,50),(300,80,100,50),(150,220,100,50)]

def obsDraw(pygame,screen):
    blue=(0,0,255)
    for o in OBS: 
      pygame.draw.rect(screen,blue,o)

def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def chooseParent(nn,newnode,nodes):
        for p in nodes:
         if checkIntersect(p,newnode,OBS) and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
          nn = p
        newnode.cost=nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
        newnode.parent=nn
        return newnode,nn

def reWire(nodes,newnode,pygame,screen):
        white = 255, 240, 200
        black = 20, 20, 40
        for i in range(len(nodes)):
           p = nodes[i]
           if checkIntersect(p,newnode,OBS) and p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
              pygame.draw.line(screen,white,[p.x,p.y],[p.parent.x,p.parent.y])  
              p.parent = newnode
              p.cost=newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) 
              nodes[i]=p  
              pygame.draw.line(screen,black,[p.x,p.y],[newnode.x,newnode.y])                    
        return nodes

def drawSolutionPath(start,goal,nodes,pygame,screen):
    pink = 200, 20, 240
    nn = nodes[0]
    for p in nodes:
       if dist([p.x,p.y],[goal.x,goal.y]) < dist([nn.x,nn.y],[goal.x,goal.y]):
          nn = p
    while (nn.x,nn.y)!=(start.x,start.y):
        pygame.draw.line(screen,pink,[nn.x,nn.y],[nn.parent.x,nn.parent.y],5)  
        nn=nn.parent

class Node:
    x = 0
    y = 0
    cost=0  
    parent=None
    def __init__(self,xcoord, ycoord):
         self.x = xcoord
         self.y = ycoord

def extend(nodes,screen,black):
  # This function is to sample a new configuration and extend the tree
  # toward that direction
  rand = Node(random.random()*XDIM, random.random()*YDIM)
  nn = nodes[0]
  for p in nodes:
    if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
      nn = p
  interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])

  newnode = Node(interpolatedNode[0],interpolatedNode[1])
  if checkIntersect(nn,newnode,OBS):
    [newnode,nn]=chooseParent(nn,newnode,nodes)
    nodes.append(newnode)

    pygame.draw.line(screen,black,[nn.x,nn.y],[newnode.x,newnode.y])
    nodes=reWire(nodes,newnode,pygame,screen)
    pygame.display.update()

    for e in pygame.event.get():
      if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
          sys.exit("Leaving because you requested it.")
  return nodes

def connect(nodes,screen,black):
  # this function is to sample a new configuration and connect the tree
  # to the sampled configuration unless there is an obstacle in between
  # the nearest node in the tree and configuration node
  rand = Node(random.random()*XDIM, random.random()*YDIM)
  nn = nodes[0]
  for p in nodes:
    if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
      nn = p

  # extend the tree till the sampled configuration
  while((nn.x,nn.y)!=(rand.x,rand.y)):
    # or till an obstacle is found in the middle
    if(checkIntersect(nn,rand,OBS)):

      interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])

      newnode = Node(interpolatedNode[0],interpolatedNode[1])
      if checkIntersect(nn,newnode,OBS):
        [newnode,nn]=chooseParent(nn,newnode,nodes)
        nodes.append(newnode)

        pygame.draw.line(screen,black,[nn.x,nn.y],[newnode.x,newnode.y])
        nodes=reWire(nodes,newnode,pygame,screen)
        pygame.display.update()

        for e in pygame.event.get():
          if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
              sys.exit("Leaving because you requested it.")
      nn = newnode
    else:
      break

  return nodes

def find_q_nearest(nodes, target):
    # a criteria to connect the trees by finding the 
    # nearest node the target node in the second tree
    q_near = nodes[0]
    ccost = 9999
    nodes_near = []
    for node in nodes:
        if dist([target.x,target.y],[node.x,node.y])<RADIUS:
            nodes_near.append(node)
    for node in nodes_near:
        if node.cost < ccost:
            q_near = copy.deepcopy(node)
            ccost = node.cost
    return q_near

def get_path(start,goal,nodes):
  # returns a path from start to goal from a list of nodes
  ret_nodes = []
  nn = nodes[0]
  for p in nodes:
     if dist([p.x,p.y],[goal.x,goal.y]) < dist([nn.x,nn.y],[goal.x,goal.y]):
        nn = p
  while nn!=start:
    ret_nodes.append(nn)
    nn=nn.parent
  return ret_nodes

def reverse_path(parent,nodes):
    # reverst the parent and child in the second tree to combine the both trees
    ret_nodes = []
    cur_parent = parent
    cur_node = nodes[0]
    while cur_node!=None:
        newnode = Node(cur_node.x,cur_node.y)
        newnode.parent = cur_parent
        cur_node = cur_node.parent
        cur_parent = newnode
        ret_nodes.append(newnode)
    return ret_nodes

def path_length(nodes):
  length = 0
  for i in range(len(nodes)-1):
    length+=dist([nodes[i].x,nodes[i].y],[nodes[i+1].x,nodes[i+1].y])
  return length

def main(imgno):
  pygame.init()
  screen = pygame.display.set_mode(WINSIZE)
  pygame.display.set_caption('Bi-Directional RRTstar')
  white = 255, 255, 255
  black = 20, 20, 40
  screen.fill(white)
  obsDraw(pygame,screen)

  t = time.time()
  nodes_from_root = []
  nodes_from_goal = []
  
  nodes = []
  nodes_from_root.append(Node(0.0,0.0)) # Start in the corner (upper left)
  nodes_from_goal.append(Node(630.0,470.0)) # Start in the corner (lower right)
  # two different starting points for two trees
  start_root=nodes_from_root[0]
  start_goal=nodes_from_goal[0]

  # corresponding goals for the two trees
  goal_root=Node(630.0,470.0)
  goal_goal=Node(0.0,0.0)
  q_nearest = None
  q_target = nodes_from_goal[0]
  for i in range(NUMNODES):
    if(i%2):
      nodes_from_root = extend(nodes_from_root,screen,black)
    else:
      nodes_from_goal = extend(nodes_from_goal,screen,black)
      q_target = nodes_from_goal[len(nodes_from_goal)-1]
    q_nearest = find_q_nearest(nodes_from_root,q_target)
    # check if the target node and nearest nodes are close enough for the
    # trees to be connected
    if(dist([q_target.x,q_target.y],[q_nearest.x,q_nearest.y])<RADIUS):
      if checkIntersect(q_nearest,q_target,OBS):
          newnode = Node(q_target.x,q_target.y)
          newnode.parent = q_nearest
          nodes_from_root.append(newnode)
          pygame.draw.line(screen,black,[q_nearest.x,q_nearest.y],[newnode.x,newnode.y])
      break
  nodes_expanded = len(nodes_from_root)+len(nodes_from_goal)

  # get the path from root to the nearest node to the target in the second tree
  pppath = get_path(start_root,q_nearest,nodes_from_root)

  # get the path from the goal to the target
  ppath = get_path(start_goal,q_target,nodes_from_goal)

  # reverse the relationship between the nodes and reverse the elements
  ppath = reverse_path(q_nearest,ppath)
  ppath.reverse()

  # add the second path to the first path
  ppath.extend(pppath)
  elapsed = time.time()-t
  path_len = path_length(ppath)

  # draw the solution
  drawSolutionPath(start_root,goal_root,ppath,pygame,screen)
  pygame.display.update()
  pygame.image.save(screen, "image"+str(imgno)+".jpg")
  print nodes_expanded,elapsed,path_len
  pygame.quit()

if __name__ == '__main__':
  for i in range(1):
    main(i)
