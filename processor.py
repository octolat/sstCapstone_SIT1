import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    corners, edges = import_shape("cube.obj")




def find_odd_nodes(edges):
    odd_edges = []
    for idx, corner in enumerate(edges):
        sum = 0
        for connected in corner:
            if connected == 1:
                sum += 1
        if sum %2 != 0:
            odd_edges.append(idx)

def import_shape(file):
    with open(file, "r") as f:
        shape = f.read().split("\n")
        shape = shape[:-1]

    #assign each corner a index + store pos
    corners = []
    for cmd in shape:
        cmd = cmd.split()
        if cmd[0] == "v":
            #x y z
            coords = cmd[1:]
            coords = list(map(float, coords))
            corners.append(coords)

    #create connectivity matrix 
    edges = []
    #row / column 0 is a dud so that we can use 1-based indexing
    for i in range(len(corners)+1):
        tmp = []
        for j in range(len(corners)+1):
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
    debug_printList(edges)
    debug_renderShape(corners, edges)
    return corners, edges

def debug_printList(list):
    for ch in list:
        print(ch)
                
def debug_renderShape(corners_list, adj_list):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    '''
    for node, (x,y,z) in enumerate(corners_list):
        ax.scatter(x, y, z, color='red', s=40)
    '''

    for y in range(len(adj_list)):
        for x in range(y+1, len(adj_list)):
            if adj_list[y][x] == 1:
                ax.plot([corners_list[y][0], corners_list[x][0]], [corners_list[y][1], corners_list[x][1]], [corners_list[y][2], corners_list[x][2]], color='blue')              
            
    ax.set_box_aspect([1,1,1])  # equal aspect ratio
    ax.grid(False)
    ax.axis('off')
    plt.show()
    

main()