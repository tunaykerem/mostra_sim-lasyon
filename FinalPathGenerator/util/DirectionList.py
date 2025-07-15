def directionList(path,waypointIndex):
    start_direction = ""
    direction = " "
    directionList=[]
    for index in range (0,len(path)-1):
        if len(path)>2:
            dX = path[index][0]-path[index+1][0]
            dY = path[index][1]-path[index+1][1]
            if index==0:
                if dX == 1:
                    direction = "K"
                elif dX == -1:
                    direction = "G"
                elif dY == 1:
                    direction = "B"
                elif dY == -1:
                    direction = "D"
                directionList.append(direction)
                start_direction= direction
            else:
                if index in waypointIndex:
                    if dX == 1:
                        direction = "K"
                    elif dX == -1:
                        direction = "G"
                    elif dY == 1:
                        direction = "B"
                    elif dY == -1:
                        direction = "D"  
                    directionList.append(direction)
    return directionList
    