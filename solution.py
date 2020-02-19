#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems
import numpy as np
def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def border_check(state):
    rval1=False
    rval2=True
    x,y=[],[]
    
    for i in state.storage:
        x=x+[i[0]]
        y=y+[i[1]]
        
    for i in state.boxes:
        if (i[0]==0 and i[0] not in x):
            rval2=False
        if (i[0]==state.width-1  and i[0] not in x):
            rval2=False
        if (i[1]==0  and i[1] not in y):
            rval2=False
        if (i[1]==state.height-1 and i[1] not in y):
            rval2=False
    return rval2

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal
    total_dist=0
    min_dist=state.width+state.height
    for i in state.boxes:
        for j in state.storage:
            dist=abs(i[0]-j[0])+abs(i[1]-j[1])
            if(dist<min_dist):
                min_dist=dist
        total_dist=total_dist+min_dist
        min_dist=state.width+state.height
    
    return total_dist
  
#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

   
def heur_alternate(state):
    total_dist=0
    min_dist=state.width+state.height
    spots_taken=[]
    min_spot=()
    boxes=list(state.boxes)
    obs=list(state.obstacles)
    if(corner_checking(state)==False or obs_checking(state)==False):
        return float('inf')

    x,y=[],[]
    for i in obs:
        x=x+[i[0]]
        y=y+[i[1]]
    
    for i in boxes:
        for j in state.storage:
            dist=abs(i[0]-j[0])+abs(i[1]-j[1])
            if(i[1] in y and j[1] in y):
                for k in obs:
                    if((i[0]<k[0] and j[0]>k[0]) or (i[0]>k[0] and j[0]<k[0])):
                        dist=dist+1
            if(i[0] in x and j[0] in x):
                for k in obs:
                    if((i[1]<k[1] and j[1]>k[1]) or (i[1]>k[1] and j[1]<k[1])):
                        dist=dist+1
              
            if(dist<min_dist and j not in spots_taken):
                min_dist=dist
                min_spot=j
        total_dist=total_dist+min_dist
        spots_taken=spots_taken+[min_spot]
        min_dist=state.width+state.height 
    
    if(state.gval<10):
        if(robot_to_box(state)<len(state.robots)):
            return robot_to_box(state)
    #return total_dist+trivial_heuristic(state)+0.7*box(state)
    return total_dist+0.7*box(state)        
def corner_checking(state):
    corners=[(0,0),(state.width-1,0),(0,state.height-1),(state.width-1,state.height-1)]
    box=[]
    s=0
    for i in state.boxes:
        if(i in corners):
            box=box+[i]
            s=s+1
    for i in state.storage:
        if i in box:
            s=s-1
    if(s>0):
        return False
    return True

def obs_checking(state):
    for i in state.boxes:
        if (i[0]>0 and i[1]>0):
            if((i[0]-1,i[1]) in state.obstacles and (i[0],i[1]-1) in state.obstacles and i not in state.storage):
                return False
        if(i[0]>0 and i[1]<(state.height-1)):
            if((i[0]-1,i[1]) in state.obstacles and (i[0],i[1]+1) in state.obstacles and i not in state.storage):
                return False
        if (i[0]<(state.width-1) and i[1]>0):
            if((i[0]+1,i[1]) in state.obstacles and (i[0],i[1]-1) in state.obstacles and i not in state.storage):
                return False
        if(i[0]<(state.width-1) and i[1]<(state.height-1)):
            if((i[0]+1,i[1]) in state.obstacles and (i[0],i[1]+1) in state.obstacles and i not in state.storage):
                return False
    return True

def box(state):
    rval=0
    for i in state.boxes:
        if (((i[0]+1,i[1]) in state.boxes or (i[0],i[1]+1) in state.boxes or (i[0]-1,i[1]) in state.boxes or (i[0],i[1]-1) in state.boxes) 
        and (i not in state.storage )):
            rval=rval+1
    return rval
def robot_to_box(state):
    total_dist=0
    min_dist=state.width+state.height
    for i in state.robots:
        for j in state.boxes:
            dist=abs(i[0]-j[0])+abs(i[1]-j[1])
            if(dist<min_dist):
                min_dist=dist
        total_dist=total_dist+min_dist
        min_dist=state.width+state.height
    return total_dist


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval+weight*sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  init_time=os.times()[0]
  time=timebound
  weight1=0
  se=SearchEngine('custom','full')
  se.init_search(initial_state,goal_fn=sokoban_goal_state,heur_fn=heur_fn,fval_function=lambda sN : fval_function(sN,weight))
  new_time=os.times()[0]
  rval=se.search(timebound-(new_time-init_time))
  if rval!=False:
      cost=(float('inf'),float('inf'),rval.gval)
      new_time=os.times()[0]
      while(new_time-init_time<timebound and not se.open.empty):
          weight1=weight1+1
          if(weight1<=weight):
              se.fval_function=lambda sN : fval_function(sN,weight-weight1)
          rval1=se.search(timebound-(new_time-init_time),cost)
          if (rval1!=False):
              rval=rval1
              new_time=os.times()[0]
              time=timebound-(new_time-init_time)
              cost=(float('inf'),float('inf'),rval1.gval)

  return rval

  
def anytime_gbfs(initial_state, heur_fn, timebound = 10):
  init_time=os.times()[0]
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  time=timebound
  se=SearchEngine('best_first','full')
  se.init_search(initial_state,goal_fn=sokoban_goal_state,heur_fn=heur_fn)
  new_time=os.times()[0]
  rval=se.search(timebound-(new_time-init_time))
  if rval!=False:
      cost=(rval.gval,float('inf'),float('inf'))
      new_time=os.times()[0]
      while(new_time-init_time<timebound and not se.open.empty):
          rval1=se.search(cost)
          if (rval1!=False):
              rval=rval1
              new_time=os.times()[0]
              time=timebound-(new_time-init_time)
              if(rval1!=False):
                  cost=(rval1.gval,float('inf'),float('inf'))

  return rval
