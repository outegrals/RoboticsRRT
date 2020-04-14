# Tommy Truong
import math
import random
from sympy import *
from queue import PriorityQueue

# Global variables that I can change when I'm being lazy
INF = 0x3f3f3f
false = False
true = True
samplingSize =1000

# Class of node
class Node:
    def __init__(self, x=None, y=None, z=None, ID=None):
        self.x = x
        self.y = y
        self.z = z
        self.ID = ID
        self.next = []
        self.dist = dist(x, y, z)
        self.nodeDist = None

def main():

    # Reading in text file for configuration space
    with open('world55.txt') as f:
        line = f.read().split('\n')

        x = line[0].split(' ')
        x = [int(i) for i in x]

        y = line[1].split(' ')
        y = [int(i) for i in y]

        z = line[2].split(' ')
        z = [int(i) for i in z]

        cube1 = line[3].split(' ')
        cube1 = [int(i) for i in cube1]

        cube2 = line[4].split(' ')
        cube2 = [int(i) for i in cube2]

        sphere1 = line[5].split(' ')
        sphere1 = [int(i) for i in sphere1]

        sphere2 = line[6].split(' ')
        sphere2 = [int(i) for i in sphere2]

        start = line[7].split(' ')
        start = [int(i) for i in start]

        end = line[8].split(' ')
        end = [int(i) for i in end]

    # initializing the world
    world = [[[0 for depth in range(z[1] + 1)]
              for col in range(y[1] + 1)] for row in range(x[1] + 1)]

    # Making a configuration space list to be passed into the other methods
    c_space = [x, y, z]
    print('The world has been created')

    # Setting the start and end value into the world
    world[start[0]][start[1]][start[2]] = 1
    world[end[0]][end[1]][end[2]] = 2

    # Adding obstacles onto the world
    side = cube1[3]/2 
    for i in range(int(cube1[0] - side), int(cube1[0] + side)):
        for j in range(int(cube1[1] - side), int(cube1[1] + side)):
            for k in range(int(cube1[2] - side), int(cube1[2] + side)):
                world[i][j][k] = -1
    side = cube2[3]/2 
    for i in range(int(cube2[0] - side), int(cube2[0] + side)):
        for j in range(int(cube2[1] - side), int(cube2[1] + side)):
            for k in range(int(cube2[2] - side), int(cube2[2] + side)):
                world[i][j][k] = -1
    side = sphere1[3]
    for i in range(int(sphere1[0] - side), int(sphere1[0] + side)):
        for j in range(int(sphere1[1] - side), int(sphere1[1] + side)):
            for k in range(int(sphere1[2] - side), int(sphere1[2] + side)):
                if distance(i, j, k, sphere1[0], sphere1[1], sphere1[2]) <= side:
                    world[i][j][k] = -1

    side = sphere2[3]
    for i in range(int(sphere2[0] - side), int(sphere2[0] + side)):
        for j in range(int(sphere2[1] - side), int(sphere2[1] + side)):
            for k in range(int(sphere2[2] - side), int(sphere2[2] + side)):
                if distance(i, j, k, sphere2[0], sphere2[1], sphere2[2]) <= side:
                    world[i][j][k] = -1
    print('All obstacles have been place in the world')

    # Making a list of the obstacles for cube and sphere
    cube = [cube1, cube2]
    sphere = [sphere1, sphere2]

    # Making a start and end node to be reference later

    startNode = Node(start[0], start[1], start[2], 0)
    endNode = Node(end[0], end[1], end[2], 1)

    while True:
        goalFlag = true
        print('Tree is being generated...')
        # Generate a tree from both the start and end node
        fromStart = generateTree(world, startNode, cube, sphere, c_space, false)
        # print(len(fromStart))
        fromGoal = generateTree(world, endNode, cube, sphere, c_space, true)
        # reverse this list so that way the goal is at the end of the list
        fromGoal.reverse()
    
        while(not any(p in fromGoal for p in fromStart)):
            # Generate a tree from both the start and end node
            fromStart = generateTree(world, startNode, cube, sphere, c_space, false)
            # print(len(fromStart))
            fromGoal = generateTree(world, endNode, cube, sphere, c_space, true)
            # reverse this list so that way the goal is at the end of the list
            fromGoal.reverse()
            print('bad tree')
        # maybe do something if it doesn't connect
        radius = 2
        
        # combine both trees
        points = fromStart + fromGoal

        nodes = []
        # converts the points into a node with the ID being the index of the list which is why fromGoal had to be reverse
        for i in range(0, len(points)):
            nodes.append(Node(points[i][0], points[i][1], points[i][2], i))
        #l = len(nodes) - 1
        #print(nodes[l].x, nodes[l].y, nodes[l].z)

        print('Making an adjanceny list of the nodes')

        # I wish this went faster but O(n^2)
        for i in range(0, len(nodes)):
            count = 0
            nodeFlag = false
            n1 = nodes[i]
            # goes through each node one at a time and compare it with all the nodes in the list
            for j in range(0, len(nodes)):
                n2 = nodes[j]
                n2.nodeDist = distance(n1.x, n1.y, n1.z, n2.x, n2.y, n2.z)
                # if the neighbors are within a distance of each other then check if there is a obstacle obstructing the edge
                # if not then add it as a neighbor
                if isNeighbor(n1, n2, radius):
                    cubeFlag = sphereFlag = false
                    for k in range(len(cube)):
                        if not validEdge(cube[k], cube[k][3]/2, n1, n2):
                            cubeFlag = true
                            break
                    if cubeFlag:
                        continue
                    for k in range(len(sphere)):
                        if not validEdge(sphere[k], sphere[k][3], n1, n2):
                            sphereFlag = True
                            break
                    if sphereFlag:
                        continue
                    if count == 5:
                        break
                    else:
                        count += 1
                    n2.dist = distance(n1.x,n1.y,n1.z,n2.x,n2.y,n2.z)
                    nodes[i].next.append(n2)

        # Using Dijsktra to find path
        print('Finding the path....')
        dist_point = [INF]*len(nodes)
        dist_point[nodes[0].ID] = 0.0
        prev = [Node]*len(nodes)
        pqueue = PriorityQueue()
        pqueue.put((nodes[0].ID, nodes[0]))
        queue = []
        queue.append(nodes[0])
        visited = [False]*len(nodes)

        
        while not pqueue.empty():
            node = pqueue.get()[1]
            if visited[node.ID]:
                continue
            if node.ID == len(nodes) - 1:
                goalFlag = false
                break
                #print(node.x, node.y , node.z)
            for i in range(0, len(node.next)):
                if dist_point[node.next[i].ID] > dist_point[node.ID] + node.next[i].dist:
                    dist_point[node.next[i].ID] = dist_point[node.ID] + node.next[i].dist
                    #print(node.x, node.y, node.z)
                    pqueue.put((node.next[i].ID, node.next[i]))
                    # queue.append(node.next[i])
                    prev[node.next[i].ID] = node
                    #print(node.x, node.y, node.z, 'going into node: ', node.next[i].x,node.next[i].y,node.next[i].z)
            visited[node.ID] = True
        
        if goalFlag:
            print('\n\nBad path produced, remaking path...\n\n')
            continue
        else:
            break

    path = []
    stuff = []
    sequence = []
    print('the path has been found')
    # backtracking to find the path
    sequence.append(nodes[len(nodes) - 1])
    #print(nodes[len(nodes) - 1].x, nodes[len(nodes) - 1].y, nodes[len(nodes) - 1].z)
    ID = nodes[len(nodes) - 1].ID
    while ID != 0:
        sequence.append(prev[ID])
        ID = prev[ID].ID

    # Poping the stack into stuff to make it in order
    while len(sequence) != 0:
        stuff.append(sequence.pop())

    # Converting stuff into path
    for i in range(0, len(stuff)):
        path.append([stuff[i].x, stuff[i].y, stuff[i].z])

    pathSpeed = TrajectoryPlanning(path)

    print('Outputting the path into RRT_output.txt')
    file = open('RRT_output.txt', 'w')
    for i in range(0, len(path)):
        file.write('%d %d %d\n' % (path[i][0], path[i][1], path[i][2]))
    file.close()

    file = open('RRT_Trajectory.txt', 'w')
    for x in pathSpeed:
        for y in x:
            for z in y:
                file.write('%f ' % (z))
        file.write('\n')
    file.close()
    print('Finish')

# false if at the same point other wise checks the distance


def isNeighbor(n1, n2, r):
    if n1.x == n2.x and n2.y == n1.y and n1.z == n2.z:
        return false
    else:
        return distance(n1.x, n1.y, n1.z, n2.x, n2.y, n2.z) < r

# method to generate tree from start or goal


def generateTree(world, p1, cube, sphere, c_space, switch):

    i = 0
    # insert the starting node to the list and point
    nodes = [p1]
    points = [[p1.x, p1.y, p1.z]]
    x = c_space[0]
    y = c_space[1]
    z = c_space[2]
    # switch the lower bound if making a tree from the goal
    # keep generating nodes until less then desire tree length
    while i < samplingSize:
        # generate random x,y,z point
        randx = random.randint(0, x[1] - 1)
        randy = random.randint(0, y[1] - 1)
        randz = random.randint(0, z[1] - 1)

        Xnew = Node(randx, randy, randz, -1)
        # skips if at a known location or already in points
        if world[randx][randy][randz] == -1 or world[randx][randy][randz] == 2 or world[randx][randy][randz] == 1:
            continue
        elif [randx, randy, randz] in points:
            continue
        else:
            # find nearest node from the newly generate node
            tmp = INF
            Xclose = None
            for j in range(len(nodes)):
                nodeDist = distance(
                    nodes[j].x, nodes[j].y, nodes[j].z, randx, randy, randz)
                if nodeDist < tmp:
                    tmp = nodeDist
                    Xclose = nodes[j]
            # check if valid edge
            cubeFlag = sphereFlag = false
            for j in range(len(cube)):
                if not validEdge(cube[j], cube[j][3]/2, Xclose, Xnew):
                    cubeFlag = true
                    break
            if cubeFlag:
                continue
            for j in range(len(sphere)):
                if not validEdge(sphere[j], sphere[j][3], Xclose, Xnew):
                    sphereFlag = True
                    break
            if sphereFlag:
                continue

            # new sampling point is one unit away from the closet node
            x1 = Xnew.x - Xclose.x
            y1 = Xnew.y - Xclose.y
            z1 = Xnew.z - Xclose.z

            mag = abs(math.floor(tmp))
            xsamp = int(math.ceil(Xclose.x + x1/mag))
            ysamp = int(math.ceil(Xclose.y + y1/mag))
            zsamp = int(math.ceil(Xclose.z + z1/mag))
            # skip if already in points to avoid duplicates
            if [xsamp, ysamp, zsamp] in points:
                continue

            # add to node and point list
            points.append([xsamp, ysamp, zsamp])

            Xsamp = Node(xsamp, ysamp, zsamp, i)
            nodes.append(Xsamp)

            i += 1


    return points


def TrajectoryPlanning(path):


    X = findCoeffiecent(path[0][0], 0, 0, path[1][0], 1, .15, 0, 1)
    Y = findCoeffiecent(path[0][1], 0, 0, path[1][1], 1, .15, 0, 1)
    Z = findCoeffiecent(path[0][2], 0, 0, path[1][2], 1, .15, 0, 1)

    traj = []
    traj.append([X,Y,Z])
    v1 = v2 = 1
    for i in range(1, len(path) - 1):
        tmp = []
        for j in range(3):
            tmp.append(findCoeffiecent(path[i][j], 1, .05, path[i + 1][j], 1, .05, i, i + 1))
        traj.append(tmp)
    
    n = len(path) - 1
    m = len(path) - 2

    X = findCoeffiecent(path[m][0], 1, .05, path[n][0], 0, 0, m, n)
    Y = findCoeffiecent(path[m][0], 1, .05, path[n][0], 0, 0, m, n)
    Z = findCoeffiecent(path[m][0], 1, .05, path[n][0], 0, 0, m, n)

    traj.append([X,Y,Z])

   # print(traj)
    
    return traj

def velocity(Coefficent, t):
    return Coefficent[1] + 2*Coefficent[2]*t + 3*Coefficent[3]*math.pow(t, 2)

def acceleration(Coeffiecent, t):
    return 2*Coeffiecent[2] + 6*Coeffiecent[3]*t

def findCoeffiecent(q0, v0, a0, q1, v1, a1, t0, t1):

    '''
    M = Matrix([[1, t0, math.pow(t0, 2), math.pow(t0, 3), q0], [0, 1, 2*t0, 3*math.pow(t0,2), v0],
                [1, t1, math.pow(t1, 2), math.pow(t1, 3), q1], [0, 1, 2*t1, 3*math.pow(t1,2), v1]])
    '''
    M = Matrix([[1, t0, math.pow(t0, 2), math.pow(t0, 3), math.pow(t0, 4), math.pow(t0, 5), q0], 
                [0, 1, 2*t0, 3*math.pow(t0,2), 4*math.pow(t0, 3), 5*math.pow(t0, 4),  v0],
                [0, 0, 2, 6*t0, 12*math.pow(t0, 2), 20*math.pow(t0, 3), a0],
                [1, t1, math.pow(t1, 2), math.pow(t1, 3), math.pow(t1, 4), math.pow(t1, 5), q1], 
                [0, 1, 2*t1, 3*math.pow(t1,2), 4*math.pow(t1, 3), 5*math.pow(t1, 4),  v1],
                [0, 0, 2, 6*t1, 12*math.pow(t1, 2), 20*math.pow(t1, 3), a1]])
    
    M_rref = M.rref()
    #print(M_rref[0])
    #print(M_rref[0].col(-1))
    coeffiecent = []
    for x in M_rref[0].col(-1):
        coeffiecent.append(x)    
    #print(coeffiecent)

    return coeffiecent


# basically some geometry but if the obstacle x,y,z is within range of the two nodes x,y,z then it's not a valid edge
def validEdge(obstacle, side, n1, n2):
    flagx = flagy = flagz = true
    if int(n1.x) in range(int(obstacle[0] - side), int(obstacle[0] + side + 1)) or int(n2.x) in range(int(obstacle[0] - side), int(obstacle[0] + side + 1)):
    #if int(obstacle[0] - side) in range(int(n1.x), int(n2.x)) or int(obstacle[0] + side + 1) in range(int(n1.x), int(n2.x)):
        #print('x: ', n1.x, n2.x, obstacle[0] - side, obstacle[0] + side)
        flagx = false
    if int(n1.y) in range(int(obstacle[1] - side), int(obstacle[1] + side + 1)) or int(n2.y) in range(int(obstacle[1] - side), int(obstacle[1] + side + 1)):
        #print('y: ',n1.y, n2.y, obstacle[1] - side, obstacle[1] + side)
        flagy = false
    if int(n1.z) in range(int(obstacle[2] - side), int(obstacle[2] + side + 1)) or int(n2.z) in range(int(obstacle[2] - side), int(obstacle[2] + side + 1)):
        #print('z: ',n1.z, n2.z, obstacle[2] - side, obstacle[2] + side)
        flagz = false
    
    return flagx or flagy or flagz

# Distance formula


def distance(x1, y1, z1, x0, y0, z0):
    x = math.pow(x1 - x0, 2)
    y = math.pow(y1 - y0, 2)
    z = math.pow(z1 - z0, 2)

    return math.sqrt(x+y+z)

# distance formula from origin
def dist(x1, y1, z1):
    x = math.pow(x1, 2)
    y = math.pow(y1, 2)
    z = math.pow(z1, 2)
    return math.sqrt(x+y+z)


main()
