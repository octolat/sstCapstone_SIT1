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
    path = [7, 8, 6, 5, 7, 3, 1, 2, 4, 3, 1, 5, 6, 2, 4, 8]
    corners_pos, edges, normals = import_shape("shapes/cube.obj")
    #edges_eulered = make_eulerian(corners_pos, edges)
    #path = heirholzer(edges_eulered)
    #print(path)
    #edges, newpath = anti_180_rerouting(corners_pos, edges_eulered, path)
    #debug_printList(edges_straight)
    #debug_renderShape(corners_pos, edges_eulered, True)
    
    
    #print(path)
    #debug_animatePath(corners_pos, path, 0.7)
    #debug_animatePath(corners_pos, newpath, 0.7)
    #print(path)
    translator(path,corners_pos,normals)

    

def translator(path,corners_pos, normals):
    instructions = ""
    old_N = rotational_axis_vec = None
    absolute_rotate = 0

    for i in range(1, len(path)):
        N_of_Normal = instruct_bend = instruct_feed = instruct_rotate = curr_N = 0

        #find points and vectors
        pt2, pt1 = path[i-1], path[i] #pt1 is curr, pt2 is previous
        v1 = find_vector(pt2,pt1, corners_pos)
        if i > 1:
            pt3 = path[i-2]
            v2 = find_vector(pt3, pt2, corners_pos)
            curr_N = find_referenceNormal(pt1,pt2,pt3,normals)

        #find feed
        instruct_feed = find_lineLength(pt1, pt2, corners_pos)

        #find bend
        if i > 1:
            instruct_bend = find_angleBetweenVec(v2,v1)
            false_normal = find_unitNormal(v2,v1)
            r = find_angleBetweenVec(false_normal, curr_N)
            if r == 0:
                instruct_bend *= 1 #clockwise
            elif r == 180:
                instruct_bend *= -1 #anti clockwise

        #find rotate
        if i > 2:
            instruct_rotate = find_angleBetweenVec(old_N, curr_N)
            absolute_rotate += instruct_rotate
            if abs(absolute_rotate) > 180:
                sign = absolute_rotate / abs(absolute_rotate)
                mag = abs(absolute_rotate) - 180
                absolute_rotate = sign * mag
                instruct_rotate = -1 * sign * (360 - instruct_rotate)

            if instruct_rotate != 0:
                N_of_Normal = find_unitNormal(curr_N, old_N)
                
                r = find_dotProduct(N_of_Normal, rotational_axis_vec)
                if r > 0:
                    instruct_rotate *= 1 # clockwise
                elif r < 0:
                    instruct_rotate *= -1 # anti-clockwise
        
        rotational_axis_vec = v1
        old_N = curr_N

        print(f"cnr: {pt1}, rotate: {instruct_rotate}, bend: {instruct_bend}, feed: {instruct_feed}")
        #make txt file
        if instruct_rotate != 0:
            if instruct_rotate > 0: sign = '+'
            else: sign = ''
            instructions += "R" + sign + str(round(instruct_rotate,1)) + "\n"
        if instruct_bend != 0:
            if instruct_bend > 0: sign = '+'
            else: sign = ''
            instructions += "B" + sign + str(round(instruct_bend,1)) + "\n"
        if instruct_feed != 0:
            if instruct_feed > 0: sign = ''
            else: sign = ''
            instructions += "F" + sign + str(round(instruct_feed*10)) + "\n"
    with open("instructions.txt", "w") as f:
        f.write(instructions)

def find_unitNormal(a,b):
    #find normal
    n = find_crossProduct(a,b)
    #make it a unit vector
    length = find_lineLength(0,1,[[0,0,0],n])
    n = [num/length for num in n]
    return n 

def find_angleBetweenVec(a,b):
    dotprd = find_dotProduct(a,b)
    dotprd /= find_lineLength(0,1,[[0,0,0],a])
    dotprd /= find_lineLength(0,1,[[0,0,0],b])
    theta = math.acos(dotprd)/math.pi*180
    if theta < 0.1: theta = 0 #if its colineear jus get clamp it
    if theta >199.9: theta = 180
    return theta

def find_crossProduct(a, b):
    #finding cross product
    new = [0,0,0]
    new[0] = (a[1]*b[2] - a[2]*b[1])
    new[1] = (-a[0]*b[2] + a[2]*b[0])
    new[2] = (a[0]*b[1] - a[1]*b[0])

    return new

def find_dotProduct(a, b):
    #finding cross product
    sum = 0
    for i in range(3):
        sum += a[i]*b[i]
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
            if neighbour > 0: #found a neighbour
                closedpath.append(idx)
                edges[curr][idx] -=1
                edges[idx][curr] -=1
                edge_travelled = True
                break
        if not edge_travelled: #theres no neightbours
            bendpath.append(curr)
            closedpath = closedpath[:-1]
    bendpath.reverse()
    return bendpath

def anti_180_rerouting(corners_pos, edges, path):
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

def find_referenceNormal(a,b,c,normals):
    tmp = [a,b,c]
    tmp.sort()
    key = ""
    for ch in tmp:
        key += str(ch)
    return normals[key]


def import_shape(file):
    with open(file, "r") as f:
        shape = f.read().split("\n")
        shape = shape[:-1]

    #---get corner pos----
    #assign each corner a index + store pos
    corners_pos = [0] #set one first to act as a dud to create 1-based indexing
    for cmd in shape:
        cmd = cmd.split()
        if cmd[0] == "v":
            #x y z
            coords = cmd[1:]
            coords = list(map(float, coords))
            corners_pos.append(coords)

    #--- make edges list---
    #create connectivity matrix 
    edges = []
    for i in range(len(corners_pos)):
        tmp = []
        for _ in range(len(corners_pos)):
            tmp.append(0)
        edges.append(tmp)

    #---make normals list---
    #temp normals reference
    face_normals = []
    for cmd in shape:
        cmd = cmd.split()   
        if cmd[0] == "vn":
            tmp = [float(x) for x in cmd[1:]]
            face_normals.append(tmp)

    #populate edge list and normal list with f (obj is 1-indexed)
    normals = {}
    for cmd in shape:
        cmd = cmd.split()   
        if cmd[0] == "f":
            cmd = cmd[1:]
            normal = face_normals[int(cmd[0].split("/")[2])-1]
            for i in range(len(cmd)):
                curr_i = i
                prevOne_i = (i-1)%len(cmd)
                prevTwo_i = (i-2)%len(cmd)
                curr = int(cmd[curr_i].split("/")[0])
                prevOne = int(cmd[prevOne_i].split("/")[0])
                prevTwo = int(cmd[prevTwo_i].split("/")[0])

                #create edge list
                edges[curr][prevOne] = 1
                edges[prevOne][curr] = 1

                #create normals face
                #reorder the edges to accending
                tmp = [curr, prevOne, prevTwo]
                tmp.sort()
                key = ""
                for ch in tmp:
                    key += str(ch)
                normals[key] = normal



    return corners_pos, edges, normals

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