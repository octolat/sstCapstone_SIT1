import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import itertools
import heapq
import random
import copy
import sys
import math

"""   Functions
import_shape(file) -> corner_pos, edges
make_eulerian(corners_pos, edges) -> edges
heirholzer(edges) -> path
anti_180_rerouting(corners_pos, edges, path) -> edges, path

dijkstra(start, end, corners_pos, edges) -> length, path

find_oddNodes(edges) -> odd_nodes(a set)
find_lineLength(a, b, corners_pos) -> length
debug_renderShape(corners_pos_list, adj_list, show_corners = False, show_path = False, delay = 0) -> None
debug_renderShape(corners_pos_list, delay = 0, show_corners = False) -> None
debug_printList(list) -> None

"""  

def main():
    '''
    corners_pos, edges = import_shape("cube.obj")
    edges_eulered = make_eulerian(corners_pos, edges)
    path = heirholzer(edges_eulered)
    edges_straight, path_straight = anti_180_rerouting(corners_pos, edges, path)
    debug_printList(edges_straight)
    debug_animatePath(corners_pos, path, 0.7)
    #debug_animatePath(corners_pos, path_straight, 0.7)
    '''
    t = [
        [0,0,0],
        [0,1,1],
        [1,1,0],
        [1,2,0]
    ]
    p = [[0,0,0],
         [2,0,0],
         [2,1,0],
         [1,0,1],
         [1,1,1],
         [0,1,0]
         ]
    p_path = [0,1,2,4,3,0,5,2,1]
    tmp = [[0,0,0],
           [5,0,0],
           [1,3,0],
           [2,1,1],
           [1,-3,0]]
    t_path = [0,1,2,3,0]
    translator(p_path,p)

def translator(path,corners_pos):
    instructions = ""
    bend_dir = 1
    rotate_dir = 1
    past_N = None
    
    for i in range(1, len(path)): #cause we going in trios
        nOfN = instruct_bend = instruct_feed = instruct_rotate = curr_N = 0

        #find feed
        pt2, pt1 = path[i-1], path[i] #pt1 is curr, pt2 is previous
        instruct_feed = find_lineLength(pt1, pt2, corners_pos)
        v1 = find_vector(pt2,pt1, corners_pos)
        if i > 1: 
            #find bend
            pt3 = path[i-2]
            v2 = find_vector(pt3,pt2, corners_pos)
            #bend magnitude
            instruct_bend = find_angleBetweenVec(v2, v1)
            #current planes normal
            curr_N = find_unitNormal(v2, v1)
        if i > 2: 
            rotate_angle = 0
            #find rotate
            instruct_rotate = find_angleBetweenVec(past_N,curr_N)
            
            

            #changing of be
            if instruct_rotate == 180: #bend changes direction (no plane change)
                instruct_rotate = 0
                bend_dir = -bend_dir
                #print()
            elif instruct_rotate != 0: #we are changing plane
                #change  
                #normal of 2 normals' plane
                nOfN = find_unitNormal(v2,v1)
                rotate_angle = find_rotaryangle(past_N,curr_N)
                #print(rotate_angle)
                if find_angleBetweenVec(nOfN, v1) == 180:
                    bend_dir = -bend_dir
            print(instruct_rotate, rotate_angle)
        if i > 1:
            past_N = curr_N #ik ts is disgusting but first node still havent find normal


        instruct_rotate *= rotate_dir
        instruct_bend *= bend_dir
        print(f"corner: {path[i]}, r:{instruct_rotate}, b:{instruct_bend}, f:{instruct_feed}")

        #make txt file
        if instruct_rotate != 0:
            instructions += "R" + str(round(instruct_rotate,1)) + "\n"
        if instruct_bend != 0:
            instructions += "B" + str(round(instruct_bend,1)) + "\n"
        instructions += "F" + str(round(instruct_feed,1)) + "\n"

    with open("instructions.txt", "w") as f:
        f.write(instructions)

    #rotate, bend, feed
    #first group, feed only   
    
def find_rotaryangle(a,b):
    normal = find_unitNormal(a,b) #find the unit normal of the normals
    a_dot_n = find_dotProduct(a,normal) # dot product of a and normal to find costheta
    b_dot_n = find_dotProduct(b,normal)
    a_proj = find_project(a,a_dot_n,normal)
    b_proj = find_project(b,b_dot_n,normal)
    cross_proj = find_crossProduct(a_proj,b_proj) #cross product them to get the 
    y = find_dotProduct(cross_proj,normal)
    x = find_dotProduct(a_proj,b_proj)
    #print(a_dot_n, b_dot_n)
    return math.atan2(y,x)/math.pi * 180 #atan2 cus it gives astc

def find_project(a,a_dot_n,n): 
    # to find projection
    # a.n gives |a|costheta cus n is unit vector, which is the shadow of the line on the normal axis
    # multiply by the n that it correlated to, so like the [x,y,z] gets multiplied back to its original
    # subtract that from the original a_x
    # so now all three axes are projected onto new plane
    a_proj = [a[i]-a_dot_n*n[i] for i in range(3)] 
    return a_proj

def find_unitNormal(z,y):
    #find normal
    n = find_crossProduct(z,y)
    #make it a unit vector
    length = find_lineLength(0,1,[[0,0,0],n])
    n = [num/length for num in n]
    return n

def find_angleBetweenVec(z,y):
    dotprd = find_dotProduct(z,y)
    dotprd /= find_lineLength(0,1,[[0,0,0],z])
    dotprd /= find_lineLength(0,1,[[0,0,0],y])
    theta = math.acos(dotprd)/math.pi*180
    if theta < 0.1: theta = 0 #if its colineear jus get clamp it
    return theta

def find_crossProduct(z, y):
    #finding cross product
    new = [0,0,0]
    new[0] = (z[1]*y[2] - z[2]*y[1])
    new[1] = (-z[0]*y[2] + z[2]*y[0])
    new[2] = (z[0]*y[1] - z[1]*y[0])

    return new

def find_dotProduct(z, y):
    #finding cross product
    sum = 0
    for i in range(3):
        sum += z[i]*y[i]
    return sum

def find_vector(a,b, corner_pos):
    #a -> b
    return [corner_pos[b][i]-corner_pos[a][i] for i in range(3)]
    

def heirholzer(edges):
    edges = copy.deepcopy(edges) #todo: 
    bendpath = []
    odd_nodes = find_oddNodes(edges)
    if len(odd_nodes) == 0: 
        start = random.randint(1, len(edges)-1)
    else:
        start = list(odd_nodes)[0]
    closedpath = [start]
    while len(closedpath) > 0:
        curr = closedpath[-1]
        edge_travelled = False
        for idx, neighbour in enumerate(edges[curr]):
            if neighbour > 0:
                closedpath.append(idx)
                edges[curr][idx] -=1
                edges[idx][curr] -=1
                edge_travelled = True
                break
        if not edge_travelled:
            bendpath.append(curr)
            closedpath = closedpath[:-1]

    return bendpath

def anti_180_rerouting(corners_pos, edges, path):
    #debug_renderShape(corners_pos, edges, True)
    print(path)
    edges = edges[:]
    path = path[:]
    iterator = 0
    while True:
        if len(path) - 3 == iterator: 
            break
        cnr_start, cnr_mid, cnr_end = path[iterator:iterator+3]
        if (cnr_start == cnr_end): #there is a 180
            #remove the edge
            cutted_edge = copy.deepcopy(edges)
            cutted_edge[cnr_start][cnr_mid] = 0
            cutted_edge[cnr_mid][cnr_start] = 0
            #cannot use the path we just came from also
            if iterator > 0:
                back = path[iterator-1]
                cutted_edge[cnr_start][back] = 0
                cutted_edge[back][cnr_start] = 0

            print(cnr_start, cnr_mid)
            debug_printList(cutted_edge)

            #disktras new thing
            _, new = list(dijkstra(cnr_start, cnr_mid, corners_pos, cutted_edge))
            N_new = len(new)
            
            # stich tgt new path
            path = path[:iterator] + new + path[iterator+2:]

            #add the i-1 and i+3 to pad then add edges
            if iterator > 0: #if its the first one its fine
                new.insert(0, path[iterator-1])
            new.append(cnr_end)
            
            for i in range(len(new)-1):
                edges[new[i]][new[i+1]] += 1
                edges[new[i+1]][new[i]] += 1
            """       
            #remove start, end edges
            edges[cnr_start][cnr_mid] -= 1
            edges[cnr_mid][cnr_start] -= 1
            

            
            #update iterator to be wjere it was
            iterator += N_new
            """

            
            debug_printList(edges)
            print(path)
            print(new)
            #debug_animatePath(corners_pos,edges,path, 0.7)
            
              
        iterator += 1
    return edges, path
            



 
def make_eulerian(corners_pos, edges):
    edges = copy.deepcopy(edges)
    odd_nodes = find_oddNodes(edges)
    N_odd = len(odd_nodes)
    N_pathsToAdd = (N_odd -2) /2 #paths with 2 odd edges can we 
    #all possible paths between all nodes
    pairs = list(itertools.combinations(odd_nodes, 2))
    #find the shortest path for all oddpair
    oddpairs = []
    for start, end in pairs:
        length, path = dijkstra(start, end, corners_pos, edges)
        oddpairs.append((length, path, (start, end)))
    oddpairs.sort()
    #add double structs and remove paths that contain added paths
    added_nodes = set()
    counter = 0
    it = 0
    while counter < N_pathsToAdd:
        if oddpairs[it][2][0] not in added_nodes and oddpairs[it][2][1] not in added_nodes:
            path = oddpairs[it][1]
            for i in range(len(path)-1):
                edges[path[i]][path[i+1]] += 1
                edges[path[i+1]][path[i]] += 1
            start, end = oddpairs[it][2]
            added_nodes.add(start)
            added_nodes.add(end)
            counter += 1
        it += 1
    return edges
        


                
        
def dijkstra(start, end, corners_pos, edges):

    #todo place pirority for edges with lesser structs (to balanace cg lowk)
    visited = [False for x in range(len(corners_pos))]
    queue = []
    length = 0
    curr = start
    path = []
    while curr != end:
        visited[curr] = True
        path.append(curr)
        for idx, neighbour in enumerate(edges[curr]):
            if neighbour > 0 and visited[idx] == False:
                #distance, idx, path
                heapq.heappush(queue, (find_lineLength(curr, idx, corners_pos) + length, idx, path.copy()))
        #pick new curr
        while True:
            length, curr, path = heapq.heappop(queue)
            if visited[curr] == False: break

    path.append(end)
    return length, path




def find_oddNodes(edges):
    odd_edges = set()
    for idx, corner in enumerate(edges):
        sum = 0
        for connected in corner:
            if connected == 1:
                sum += 1
        if sum %2 != 0:
            odd_edges.add(idx)
    return odd_edges

def find_lineLength(a, b, corners_pos):
    #x y z
    x = abs(corners_pos[a][0] - corners_pos[b][0])
    z = abs(corners_pos[a][1] - corners_pos[b][1])
    y = abs(corners_pos[a][2] - corners_pos[b][2])
    diagonal = (x**2 + y**2 )**0.5
    length = (diagonal**2 + z**2)**0.5
    return length

def import_shape(file):
    with open(file, "r") as f:
        shape = f.read().split("\n")
        shape = shape[:-1]

    #assign each corner a index + store pos
    corners_pos = [0] #set one first to act as a dud to create 1-based indexing
    for cmd in shape:
        cmd = cmd.split()
        if cmd[0] == "v":
            #x y z
            coords = cmd[1:]
            coords = list(map(float, coords))
            corners_pos.append(coords)


    #create connectivity matrix 
    edges = []
    for i in range(len(corners_pos)):
        tmp = []
        for j in range(len(corners_pos)):
            tmp.append(0)
        edges.append(tmp)

    #populate with f (obj is 1-indexed)
    for cmd in shape:
        cmd = cmd.split()   
        if cmd[0] == "f":
            cmd = cmd[1:]
            for i in range(len(cmd)):
                curr = int(cmd[i])
                if i == 0: prev = int(cmd[-1])
                else: prev = int(cmd[i-1])
                edges[curr][prev] = 1
                edges[prev][curr] = 1
    return corners_pos, edges

def debug_printList(list):
    for ch in list:
        print(ch)
                
def debug_renderShape(corners_pos_list, adj_list, show_corners = False, show_path = False):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1,1,1])  # equal aspect ratio
    ax.grid(False)
    ax.axis('off')

    if show_corners:
        for node, coords in enumerate(corners_pos_list):
            if node > 0: #first index is a dud
                x,y,z = coords
                ax.scatter(x, y, z, color='red', s=40)
                ax.text(x, y, z, str(node), color='black')

    highlight_edges = []           
    if show_path != False:
        for i in range(len(show_path)-1):
            next = i+1
            highlight_edges.append([min(show_path[i], show_path[next]), max(show_path[i], show_path[next])])

    for y in range(len(adj_list)):
        for x in range(y+1, len(adj_list)):
            print(adj_list[y][x])
            if adj_list[y][x] > 0:
                print("yay")
                width = 2
                if adj_list[y][x] == 1: color = "blue"
                elif adj_list[y][x] == 2: 
                    color = "green"
                    width *= 2
                elif adj_list[y][x] == 3: 
                    color = 'yellow'
                    width *= 3
                if [y, x] in highlight_edges: color = "red"
                ax.plot([corners_pos_list[y][0], corners_pos_list[x][0]], [corners_pos_list[y][1], corners_pos_list[x][1]], [corners_pos_list[y][2], corners_pos_list[x][2]], color=color, linewidth=width)              
    plt.show()

    
def debug_animatePath(corners_pos_list, path, delay, show_corners = True):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1,1,1])  # equal aspect ratio
    ax.grid(False)
    ax.axis('off')
    plt.ion

    if show_corners:
        for node, coords in enumerate(corners_pos_list):
            if node > 0: #first index is a dud
                x,y,z = coords
                ax.scatter(x, y, z, color='red', s=40)
                ax.text(x, y, z, str(node), color='black')

    order = []
    for i in range(len(path)-1):
        next = i+1
        order.append([min(path[i], path[next]), max(path[i], path[next])])

    count = {}
    for a, b in order:

        if (a,b) not in count:
            count[(a,b)] = 1
        else:
            count[(a,b)] += 1
        width = 2 * count[(a,b)]
        ax.plot([corners_pos_list[a][0], corners_pos_list[b][0]], [corners_pos_list[a][1], corners_pos_list[b][1]], [corners_pos_list[a][2], corners_pos_list[b][2]], color="blue", linewidth = width)
        plt.draw()
        plt.pause(delay)
    input()


main()