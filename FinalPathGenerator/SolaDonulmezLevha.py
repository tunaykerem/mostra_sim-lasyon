from .util import Global as g
from FinalPathGenerator.PathGenerator import get_firstTrajectory

def solaDonulmez(carIndex,direction):
    if direction=="K":
            for i in g.corner_point:
                if carIndex[0]-1==i[0] and carIndex[1]==i[1]:
                    g.matrix[i[0],i[1]-1]=0
        
    elif direction=="G":
        for i in g.corner_point:
            if carIndex[0]+1==i[0] and carIndex[1]==i[1]:
                g.matrix[i[0],i[1]+1]=0
                
    elif direction=="D":
        for i in g.corner_point:
            if carIndex[1]+1==i[1] and carIndex[0]==i[0]:
                g.matrix[i[0]-1,i[1]]=0
                
    elif direction=="B":
        for i in g.corner_point:
            if carIndex[1]-1==i[1] and carIndex[0]==i[0]:
                g.matrix[i[0]+1,i[1]]=0

    g.x_main,g.y_main=get_firstTrajectory()