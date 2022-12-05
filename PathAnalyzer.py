import networkx as nx
import matplotlib.pyplot as plt
from DijkstrasWithExplored import dijkstra_explored
from BreadthFirstSearch import breadthFirstSearchWithExplored
from DepthFirstSearch  import depthFirstSearchWithExplored
from AStarWithExplored import aStarWithExplored
from scipy.spatial import distance
import time

def generateGraphFromShapefile(filename):
    print(f"Loading Shapefile...")
    plt.rcParams["figure.figsize"] = (50,20)
    #conda activate OSMNX
    G = nx.read_shp(filename)
    maxX, minX, maxY, minY= None,None,None,None, None, None
    averageX = 0
    averageY = 0
    nodesList = list(G.nodes())
    for i in range(len(nodesList)):
        averageX += nodesList[i][0]
        averageY += nodesList[i][1]
        if maxX == None or maxX < nodesList[i][0]:
            maxX = nodesList[i][0]
            endNode = i
        if maxY == None or maxY < nodesList[i][1]:
            maxY = nodesList[i][1]
        if minX == None or minX > nodesList[i][0]:
            minX = nodesList[i][0]
            startNode = i
        if minY == None or minY > nodesList[i][1]:
            minY = nodesList[i][1]

    averageX = averageX / len(nodesList)
    averageY = averageY / len(nodesList)

    print(f"Edges: {G.number_of_edges()}")
    print(f"Nodes: {G.number_of_nodes()}")
    print(f"Changing shapefile to graph...")
    start_time = time.time()
    pos = {k: v for k,v in enumerate(G.nodes())}
    X=nx.Graph() #Empty graph
    X.add_nodes_from(pos.keys()) #Add nodes preserving coordinates
    l=[set(x) for x in G.edges()] #To speed things up in case of large objects
    edg=[tuple(k for k,v in pos.items() if v in sl) for sl in l] #Map the G.edges start and endpoints onto pos
    newedg = []
    for e in edg:
        if len(e) >= 2:
            newedg.append(e)
    X.add_edges_from(newedg)
    print(f"Finished creating graph in {time.time() - start_time}")
    return X, pos, minX, maxX, minY, maxY, averageX, averageY 

def plotMap(X, path,explored, pos, minX, maxX, minY, maxY):
    print("Done adding edges")
    nx.draw_networkx_nodes(X,pos,node_size=1,node_color='k')
    nx.draw_networkx_edges(X,pos, width=1, edge_color='k')
    nx.draw_networkx_nodes(explored, pos, node_size=1, node_color = "r")
    nx.draw_networkx_edges(explored,pos, width=1, edge_color = "r")

    nx.draw_networkx_nodes(path, pos, node_size=1, node_color='g')
    nx.draw_networkx_edges(path,pos, width=1, edge_color='g')
    startGraph = nx.Graph()
    startGraph.add_node(list(path)[0])
    nx.draw_networkx_nodes(startGraph,pos, node_size=1, node_color='b')
    print("Done Drawing Edges")
    plt.xlim(minX, maxX) #This changes and is problem specific
    plt.ylim(minY, maxY) #This changes and is problem specific
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('From shapefiles to NetworkX')
    plt.show()

def generateGraphFromPath(path, nodesList):
    pathGraph = nx.Graph()
    for i,node in enumerate(list(path)):
        pathGraph.add_node(nodesList[node])
        if i + 1 < len(list(path)):
            pathGraph.add_edge(nodesList[node], nodesList[path[i + 1]])
    return pathGraph


def findStartAndEnd(nodesList, averageX, averageY, maxX, maxY, minX, minY):
    return minX, maxX

def runTests(filename, startnode, endnode):
    graph, pos, minX, maxX, minY, maxY, averageX, averageY= generateGraphFromShapefile(filename)
    nodesList = list(graph.nodes())
    startnode, endnode = findStartAndEnd(nodesList, averageX, averageY, maxX, maxY, minX, minY)

    # Test AStar
    print("Running aStar")
    start_time = time.time()
    aStarpath, aStarExploredGraph = aStarWithExplored(graph, nodesList[startnode], nodesList[endnode], distance.euclidean, pos)
    aStarTime = time.time() - start_time
    aStarpathGraph = generateGraphFromPath(aStarpath, nodesList)
    plotMap(graph, aStarpathGraph, aStarExploredGraph, pos,minX,maxX,minY,maxY)

    # Test dijkstras
    print("Running dijkstras")
    start_time = time.time()
    dijkstraspath, dijkstrasExploredGraph = dijkstra_explored(graph, nodesList[startnode], nodesList[endnode], pos)
    dijkstrasTime = time.time() - start_time
    dijkstraspathGraph = generateGraphFromPath(dijkstraspath, nodesList)
    plotMap(graph, dijkstraspathGraph, dijkstrasExploredGraph, pos,minX,maxX,minY,maxY)

    # Test bfs 
    print("Running BFS")
    start_time = time.time()
    bfspath, bfsExploredGraph = breadthFirstSearchWithExplored(graph, nodesList[startnode], nodesList[endnode], pos)
    bfsTime = time.time() - start_time
    bfspathGraph = generateGraphFromPath(bfspath, nodesList)
    plotMap(graph, bfspathGraph, bfsExploredGraph, pos,minX,maxX,minY,maxY)


    # Test dfs
    print("Running DFS")
    start_time = time.time()
    dfspath, dfsExploredGraph = depthFirstSearchWithExplored(graph, nodesList[startnode], nodesList[endnode], pos)
    dfsTime = time.time() - start_time
    dfspathGraph = generateGraphFromPath(dfspath, nodesList)
    plotMap(graph, dfspathGraph, dfsExploredGraph, pos,minX,maxX,minY,maxY)

    print(f"aStar Time: {aStarTime}")
    print(f"Dijkstras Time: {dijkstrasTime}")
    print(f"BFS Time: {bfsTime}")
    print(f"DFS Time: {dfsTime}")



def main():
    '''
    ottawafilename = "OttawaData/Smaller_Roads_V3.shp"
    ottawaStartnode = 123
    ottawaEndnode = 5123
    runTests(ottawafilename, ottawaStartnode, ottawaEndnode)
    '''

    '''
    torontofilename = "TorontoData/Toronto_Road_Explort.shp"
    torontoStartnode = 123
    torontoEndnode = 5123
    runTests(torontofilename, torontoStartnode, torontoEndnode)
    '''

    '''
    manhattanfilename = "ManhattanData2/ManhattanData2.shp"
    manhattanStartnode = 123
    manhattanEndnode = 4000
    runTests(manhattanfilename, manhattanStartnode, manhattanEndnode)
    '''

    nycfilename = "NYCData2/NYCData2.shp"
    nycStartnode = 123
    nycEndnode = 5123
    runTests(nycfilename, nycStartnode, nycEndnode)



if __name__ == "__main__":
    main()