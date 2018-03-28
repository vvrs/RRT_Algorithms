#!/usr/bin/env python

# rrtstar.py
# This program generates a 
# asymptotically optimal rapidly exploring random tree (RRT* proposed by Sertac Keraman, MIT) in a rectangular region.
#
# Originally written by Steve LaValle, UIUC for simple RRT in
# May 2011
# Modified by Md Mahbubur Rahman, FIU for RRT* in
# January 2016

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
from lineIntersect import *
import copy
import pdb
# random.seed(0)

#constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 2000
RADIUS=15
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
    print nn.x,nn.y
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
  rand = Node(random.random()*XDIM, random.random()*YDIM)
  nn = nodes[0]
  for p in nodes:
    if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
      nn = p
  interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])

  newnode = Node(interpolatedNode[0],interpolatedNode[1])
  if checkIntersect(nn,newnode,OBS):
    # if(dir):
    [newnode,nn]=chooseParent(nn,newnode,nodes)
    nodes.append(newnode)
    # else:
    #     [nn,newnode]=chooseParent(newnode,nn,nodes)
    #     nodes.append(nn)

    pygame.draw.line(screen,black,[nn.x,nn.y],[newnode.x,newnode.y])
    # nodes=reWire(nodes,newnode,pygame,screen)
    pygame.display.update()
    # print i, "    ", nodes

    for e in pygame.event.get():
      if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
          sys.exit("Leaving because you requested it.")
  return nodes

def find_q_nearest(nodes, target):
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



def main():
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRTstar')
    white = 255, 255, 255
    black = 20, 20, 40
    screen.fill(white)
    obsDraw(pygame,screen)
    nodes_from_root = []
    nodes_from_goal = []
    
    #nodes.append(Node(XDIM/2.0,YDIM/2.0)) # Start in the center
    # nodes_from_root.append(Node(XDIM/2.0,YDIM/2.0)) # Start in the corner
    nodes = []
    nodes_from_root.append(Node(0.0,0.0)) # Start in the corner
    nodes_from_goal.append(Node(630.0,470.0)) # Start in the corner
    start_root=nodes_from_root[0]
    start_goal=nodes_from_goal[0]
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
      if(dist([q_target.x,q_target.y],[q_nearest.x,q_nearest.y])<RADIUS):
        if checkIntersect(q_nearest,q_target,OBS):
            newnode = Node(q_target.x,q_target.y)
            newnode.parent = q_nearest
            nodes_from_root.append(newnode)
            pygame.draw.line(screen,black,[q_nearest.x,q_nearest.y],[newnode.x,newnode.y])
        break
    pppath = get_path(start_root,q_nearest,nodes_from_root)
    ppath = get_path(start_goal,q_target,nodes_from_goal)
    ppath = reverse_path(q_nearest,ppath)
    ppath.reverse()
    ppath.extend(pppath)
    drawSolutionPath(start_root,goal_root,ppath,pygame,screen)
    pygame.display.update()


# if python says run, then we should run
if __name__ == '__main__':
    main()
    running = True
    while running:
       for event in pygame.event.get():
           if event.type == pygame.QUIT:
                 running = False