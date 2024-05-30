import numpy as np
import pandas as pds
import copy
np.set_printoptions(threshold=np.inf)

def perception(map,row,col,flag):
    array = np.array([False, False, False, False])

    # The maximun positions of the map
    superiorBorder = 0
    inferiorBorder = ((map.shape)[0]) - 1
    leftBorder = 0
    rightBorder = ((map.shape)[1]) - 1
    
    # The 4(8) directions we can go
    up = row - 1
    down = row + 1
    left = col - 1
    right = col + 1

    #Checks if  moving up will cause an IndexOutOfBound Exception if not ,he will check if there is an obstacle
    #if no obstacle is found it'll make the  movement possible
    if(up >= superiorBorder): # up
        if(map[up][col] == 0):
            array[0] = True
        if(map[up][col] != -1.0 and flag):
            array[0] = True

    if(right <= rightBorder): # R
        if(map[row][right] == 0):
            array[1] = True
        if(map[row][right] != -1.0 and flag):
            array[1] = True

    if(down <= inferiorBorder): # bot
        if(map[down][col] == 0):
            array[2] = True
        if(map[down][col] != -1.0 and flag):
            array[2] = True

    if(left >= leftBorder): # L
        if(map[row][left] == 0):
            array[3] = True
        if(map[row][left] != -1.0 and flag):
            array[3] = True

    return array

def updateMatrix(map,row,col):
    perceptionArray = perception(map,row,col, False)

    #The 4 directions we can go
    up = row - 1
    down = row + 1
    left = col - 1
    right = col + 1
    list = []

    #If possible,the value of the direction we're gonna move will update with the current value of our square +1
    if(perceptionArray[0]): # up
        map[up][col] = map[row][col]+1
        list.append(up)
        list.append(col)
    if(perceptionArray[1]): # R
        map[row][right] = map[row][col]+1
        list.append(row)
        list.append(right)
    if(perceptionArray[2]): # bot
        map[down][col] = map[row][col]+1
        list.append(down)
        list.append(col)
    if(perceptionArray[3]): # L
        map[row][left] = map[row][col]+1
        list.append(row)
        list.append(left)
    #the list return the coordinates of the squares we can move to
    return list

def completeCoefficientsMatrix(map,rowGoal,colGoal):
    newPoints=updateMatrix(map,rowGoal,colGoal)
    while(len(newPoints)>0):
        row=newPoints.pop(0)
        col=newPoints.pop(0)
        newPoints.extend(updateMatrix(map,row,col))

def obstaclesCoefficients(obstacleMap):
    prevMat = np.zeros_like(obstacleMap)

    while((prevMat != obstacleMap).any()):
        prevMat = obstacleMap
        for i in range(nrows):
            for j in range(ncols):
                if obstacleMap[i][j] != -1:
                    minVal = min(obstacleMap[i-1, j-1], obstacleMap[i-1, j], obstacleMap[i-1, j+1],
                                 obstacleMap[i, j-1],                      obstacleMap[i, j+1],
                                 obstacleMap[i+1, j-1], obstacleMap[i+1, j], obstacleMap[i+1, j+1])
                    obstacleMap[i][j] = minVal + 1
    obstacleMap[obstacleMap != -1] += 1
    


def waveFront(map,StartingRow,StartingCol,EndingRow,EndingCol):
    obstacleMap = copy.deepcopy(map)
    obstacleMap[EndingRow, EndingCol] = 0
    filterMap = copy.deepcopy(obstacleMap)
    filterMap[filterMap == 0] = 1
    filterMap[filterMap == -1] = 0
    
    print("\n -----Obstacle Matrix-------")
    obstaclesCoefficients(obstacleMap)
    print(obstacleMap)

    print("\n -----Coefficients Matrix-------")
    completeCoefficientsMatrix(map,EndingRow,EndingCol)
    print(map)

    print('\n ----- Final Weigth Map------------')
    # obstacleFuncion = lambda t: 4 ** 3 - t ** 3
    # map += np.multiply(np.array([obstacleFuncion(i) for i in obstacleMap]), filterMap)
    print(map)

    loopOn = False
    row = StartingRow
    col = StartingCol
    map[row][col] = -1
    msj = ''
    visitedListRow = [StartingRow]
    visitedListCol = [StartingCol]
    while( not loopOn):
        list = []
        listPercepcion = perception(map, row, col, True)
        up = row - 1
        down = row + 1
        left = col - 1
        right = col + 1
        
        if (listPercepcion[1]): # R
            list.append(map[row][right])
        else:
            list.append(0.0)
        if (listPercepcion[3]): # L
            list.append(map[row][left])
        else:
            list.append(0.0)
        if (listPercepcion[0]): # up 
            list.append(map[up][col])
        else:
            list.append(0.0)
        if (listPercepcion[2]): # bot
            list.append(map[down][col])
        else:
            list.append(0.0)
        
        loopOn = (row == EndingRow and col == EndingCol)
        maxValInd = pds.Series(list).idxmax()
        if list[maxValInd] <= 0:
            visitedInd += 1
            row = visitedListRow[-visitedInd]
            col = visitedListCol[-visitedInd]
            print('Went back to: [' + str(row) + ', ' + str(col) + ']')
        else:
            visitedInd = 0
            if (maxValInd == 0): # R
                msj = msj + "--->["+str(col)+"]["+str(10-row)+"]"
                col = right
            if (maxValInd == 1): # L
                msj = msj + "--->["+str(col)+"]["+str(10-row)+"]"
                col = left
            if (maxValInd == 2): # up
                msj = msj + "--->["+str(col)+"]["+str(10-row)+"]"
                row = up
            if (maxValInd == 3): # bot
                msj = msj + "--->["+str(col)+"]["+str(10-row)+"]"
                row = down
            map[row][col] = -100
            visitedListCol.append(col)
            visitedListRow.append(row)
            print("\n -----map-------")
            print(map)
    cells = np.stack((visitedListRow, visitedListCol), axis=-1)
    np.savetxt('map_order_rover.csv', cells, delimiter=',', header='row,col')
    print("\n -----Path-------")
    print(msj)


print("------ Map Creation ---------")
obstacle = True
default = input("Do you want to use the default tello map dimension(11x11) and obstacles? y/n: ")
if(default == 'y'):
    obstacle = False
    nrows = 11
    ncols = 11
    map = np.zeros((nrows,ncols))
    map[0, :] = -1
    map[-1, :] = -1
    map[:, 0] = -1
    map[:, -1] = -1 
    map[3:6, 3:8] = -1
    """
    nrows = 11
    ncols = 11
    map = np.zeros((nrows,ncols))
    map[0, :] = -1
    map[-1, :] = -1
    map[:, 0] = -1
    map[:, -1] = -1
    map[4, :] = -1
    map[4, 9] = 0
    map[5, 5] = -1
    map[6, 5] = -1
    map[7, 5] = -1
    map[8, 5] = -1
    """
else:
    obstacle = False
    nrows = 11
    ncols = 11
    map = np.zeros((nrows,ncols))
    map[0, :] = -1
    map[-1, :] = -1
    map[:, 0] = -1
    map[:, -1] = -1 

# print("\n------Start point------")
rowS = 1 #int(input("Row of start point----->"))
colS = 1 #int(input("Col of start point----->"))

# print("\n------End point------")
rowE = 9 #int(input("Row of End point----->"))
colE = 9 #int(input("Col of End point----->"))
map[rowE][colE] = 1
print("\n -----Map-------")
print(np.matrix(map))
waveFront(map,rowS,colS,rowE,colE)