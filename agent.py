"""
An AI player for Othello. 
"""

import random
import sys
import time
cache={}
# You can use the functions in othello_shared to write your AI
from othello_shared import find_lines, get_possible_moves, get_score, play_move

def eprint(*args, **kwargs): #you can use this for debugging, as it will print to sterr and not stdout
    print(*args, file=sys.stderr, **kwargs)
    
# Method to compute utility value of terminal state
def compute_utility(board, color):
    util=get_score(board)
    if color==1:
        return util[0]-util[1]
    elif color==2:
        return util[1]-util[0]

# Better heuristic value of board
def compute_heuristic(board, color): #not implemented, optional
    #IMPLEMENT
    return 0 #change this!

############ MINIMAX ###############################
def minimax_min_node(board, color, limit, caching = 0):
    
    m=get_possible_moves(board,color%2+1)
    rval=((),float('inf'))
    if(len(m)==0 or limit==0):
        return ((0,0),compute_utility(board,color))
    for i in m:
        temp=play_move(board,color%2+1,i[0],i[1])
        if caching==0:
            heuristic=minimax_max_node(temp,color,limit-1,caching)[1]
        else:
            if temp in cache:
                heuristic=cache[temp]
            else:
                heuristic=minimax_max_node(temp,color,limit-1,caching)[1]
        temp_rval=(i,heuristic)
        if(temp_rval[1]<rval[1]):
            rval=temp_rval
    if(caching == 1):
        cache[board]=rval[1]
    return rval

def minimax_max_node(board, color, limit, caching = 0): #returns highest possible utility
    m=get_possible_moves(board,color)
    rval=((),-1*float('inf'))
    if(len(m)==0 or limit==0):
        return ((0,0),compute_utility(board,color))
    for i in m:
        temp=play_move(board,color,i[0],i[1])
        if caching==0:
            heuristic=minimax_min_node(temp,color,limit-1,caching)[1]
        else:
            if temp in cache:
                heuristic=cache[temp]
            else:
                heuristic=minimax_min_node(temp,color,limit-1,caching)[1]
        temp_rval=(i,heuristic)
        if(temp_rval[1]>rval[1]):
            rval=temp_rval
    if (caching == 1):
        cache[board]=rval[1]
    return rval

def select_move_minimax(board, color, limit, caching = 0):
    cache.clear()
    return minimax_max_node(board, color, limit,caching)[0]

############ ALPHA-BETA PRUNING #####################
def alphabeta_min_node(board, color, alpha, beta, limit, caching = 0, ordering = 0):
    m=get_possible_moves(board,color%2+1)
    if(len(m)==0 or limit==0):
        return ((0,0),compute_utility(board,color))
    rval=((),float('inf'))
    if ordering==1:
        m=sorted(m,key=lambda move: compute_utility(play_move(board,color%2+1,move[0],move[1]),color))
    for i in m:
        temp=play_move(board,color%2+1,i[0],i[1])
        if caching==0:
            heuristic=alphabeta_max_node(temp,color,alpha,beta,limit-1,caching,ordering)[1]
        else:
            if temp in cache:
                heuristic=cache[temp]
            else:
                heuristic=alphabeta_max_node(temp,color,alpha,beta,limit-1,caching,ordering)[1]
            
        temp_rval=(i,heuristic)
        if(temp_rval[1]<rval[1]):
            rval=temp_rval
        if(beta>rval[1]):
            beta=rval[1]
            if beta<=alpha:
                break
    if(caching==1):
        cache[board]=rval[1]
    return rval

def alphabeta_max_node(board, color, alpha, beta, limit, caching = 0, ordering = 0):
    m=get_possible_moves(board,color)
    if(len(m)==0 or limit==0):
        return ((0,0),compute_utility(board,color))
    rval=((),-1*float('inf'))
    if ordering==1:
        m=sorted(m,key=lambda move: -1*compute_utility(play_move(board,color,move[0],move[1]),color))
    for i in m:
        temp=play_move(board,color,i[0],i[1])
        if caching==0:
            heuristic=alphabeta_min_node(temp,color,alpha,beta,limit-1,caching,ordering)[1]
        else:
            if temp in cache:
                heuristic=cache[temp]
            else:
                heuristic=alphabeta_min_node(temp,color,alpha,beta,limit-1,caching,ordering)[1]
            
        temp_rval=(i,heuristic)
        if(temp_rval[1]>rval[1]):
            rval=temp_rval
        if(alpha<rval[1]):
            alpha=rval[1]
            if beta<=alpha:
                break
    if(caching == 1):
        cache[board]=rval[1]
    return rval

def select_move_alphabeta(board, color, limit, caching = 0, ordering = 0):
    cache.clear()
    """
    Given a board and a player color, decide on a move. 
    The return value is a tuple of integers (i,j), where
    i is the column and j is the row on the board.  

    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enfoce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic 
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.    
    If ordering is ON (i.e. 1), use node ordering to expedite pruning and reduce the number of state evaluations. 
    If ordering is OFF (i.e. 0), do NOT use node ordering to expedite pruning and reduce the number of state evaluations. 
    """
    return alphabeta_max_node(board,color,-1*float('inf'),float('inf'),limit,caching,ordering)[0]

####################################################
def run_ai():
    """
    This function establishes communication with the game manager.
    It first introduces itself and receives its color.
    Then it repeatedly receives the current score and current board state
    until the game is over.
    """
    print("Othello AI") # First line is the name of this AI
    arguments = input().split(",")
    
    color = int(arguments[0]) #Player color: 1 for dark (goes first), 2 for light. 
    limit = int(arguments[1]) #Depth limit
    minimax = int(arguments[2]) #Minimax or alpha beta
    caching = int(arguments[3]) #Caching 
    ordering = int(arguments[4]) #Node-ordering (for alpha-beta only)

    if (minimax == 1): eprint("Running MINIMAX")
    else: eprint("Running ALPHA-BETA")

    if (caching == 1): eprint("State Caching is ON")
    else: eprint("State Caching is OFF")

    if (ordering == 1): eprint("Node Ordering is ON")
    else: eprint("Node Ordering is OFF")

    if (limit == -1): eprint("Depth Limit is OFF")
    else: eprint("Depth Limit is ", limit)

    if (minimax == 1 and ordering == 1): eprint("Node Ordering should have no impact on Minimax")

    while True: # This is the main loop
        # Read in the current game status, for example:
        # "SCORE 2 2" or "FINAL 33 31" if the game is over.
        # The first number is the score for player 1 (dark), the second for player 2 (light)
        next_input = input()
        status, dark_score_s, light_score_s = next_input.strip().split()
        dark_score = int(dark_score_s)
        light_score = int(light_score_s)

        if status == "FINAL": # Game is over.
            print
        else:
            board = eval(input()) # Read in the input and turn it into a Python
                                  # object. The format is a list of rows. The
                                  # squares in each row are represented by
                                  # 0 : empty square
                                  # 1 : dark disk (player 1)
                                  # 2 : light disk (player 2)

            # Select the move and send it to the manager
            if (minimax == 1): #run this if the minimax flag is given
                movei, movej = select_move_minimax(board, color, limit, caching)
            else: #else run alphabeta
                movei, movej = select_move_alphabeta(board, color, limit, caching, ordering)
            
            print("{} {}".format(movei, movej))

if __name__ == "__main__":
    run_ai()
