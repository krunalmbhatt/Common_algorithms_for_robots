import time

def segment_pair_intersection(line1, line2):
    '''Return the intersection point between 2 line segments or None, if they do not intersect. 

    arguments:
    line1 - is the first line segment and has the following form ((x1,y1), (x2,y2)) where x1, y1 is
      the first point of the line segment and x2,y2 is the second point of the line segment.
    line2 - is the second line segment and has the same format as line1.

    return:
    ipoint - A tuple (x,y) which is the point of intersection or None if it does not exist 
    '''
    ### YOUR CODE STARTS HERE ###
    intersect = True 

    if intersect:
        # intersection point between line1 and line2
        return (0,0)
    else:
        return None

    ### YOUR CODE ENDS HERE ###

def efficient_intersections(L1, L2):
    #This is the Bonus part of the assignment 

    ### YOUR CODE STARTS HERE ###
    return ([], []) 
    ### YOUR CODE ENDS HERE ###

def all_pairs_intersections(L1, L2):
    x = []
    y = []
    for l1 in L1: 
        for l2 in L2: 
          point =  segment_pair_intersection(l1, l2)
          if point: 
            x.append(point[0])
            y.append(point[1])

    return (x,y)