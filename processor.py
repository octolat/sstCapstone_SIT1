import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import itertools
import heapq

def main(): 
    corners_pos, edges = import_shape("bunny.obj")
    length, path = dijkstra(5,8,corners_pos,edges)
    print(length, path)
    debug_renderShape(corners_pos, edges, False, path)
    
    #make_eulerian(corners_pos, edges)
    


 
def make_eulerian(corners_pos, edges):
    odd_nodes = find_odd_nodes(edges)
    N_odd = len(odd_nodes)
    N_pathsToAdd = (N_odd -2) /2 #paths with 2 odd edges can we 
    #all possible paths between all nodes
    pairs = list(itertools.combinations(odd_nodes, 2))
    #find the shortest path for all oddpair
    for start, end in pairs:
        pass

                
        
def dijkstra(start, end, corners_pos, edges):
    visited = [False for x in range(len(corners_pos))]
    queue = []
    length = 0
    curr = start
    path = []
    while curr != end:
        visited[curr] = True
        path.append(curr)
        for idx, neighbour in enumerate(edges[curr]):
            if neighbour == True and visited[idx] == False:
                #distance, idx, path
                heapq.heappush(queue, (find_distance_between_two_corners(curr, idx, corners_pos) + length, idx, path.copy()))
        #pick new curr
        while True:
            length, curr, path = heapq.heappop(queue)
            if visited[curr] == False: break

    path.append(end)
    return length, path




def find_odd_nodes(edges):
    odd_edges = []
    for idx, corner in enumerate(edges):
        sum = 0
        for connected in corner:
            if connected == 1:
                sum += 1
        if sum %2 != 0:
            odd_edges.append(idx)
    return odd_edges

def find_distance_between_two_corners(a, b, corners_pos):
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
    corners_pos = [0]
    for cmd in shape:
        cmd = cmd.split()
        if cmd[0] == "v":
            #x y z
            coords = cmd[1:]
            coords = list(map(float, coords))
            corners_pos.append(coords)

    #create connectivity matrix 
    edges = []
    #row / column 0 is a dud so that we can use 1-based indexing
    for i in range(len(corners_pos)+1):
        tmp = []
        for j in range(len(corners_pos)+1):
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
    if show_corners:
        for node, coords in enumerate(corners_pos_list):
            if node > 0: #first index is a dud
                x,y,z = coords
                ax.scatter(x, y, z, color='red', s=40)
                ax.text(x, y, z, str(node), color='black')
                
    if show_path != False:
        highlight_edges = []
        for i in range(len(show_path)-1):
            next = i+1
            highlight_edges.append([min(show_path[i], show_path[next]), max(show_path[i], show_path[next])])
        print(highlight_edges)

    for y in range(len(adj_list)):
        for x in range(y+1, len(adj_list)):
            if adj_list[y][x] == 1:
                color = "blue"
                if show_path != False and [y, x] in highlight_edges: color = "red"
                ax.plot([corners_pos_list[y][0], corners_pos_list[x][0]], [corners_pos_list[y][1], corners_pos_list[x][1]], [corners_pos_list[y][2], corners_pos_list[x][2]], color=color)              
            
    ax.set_box_aspect([1,1,1])  # equal aspect ratio
    ax.grid(False)
    ax.axis('off')
    plt.show()
    

main()