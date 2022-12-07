import networkx as nx
import matplotlib.pyplot as plt
from DijkstrasWithExplored import dijkstra_explored
from BreadthFirstSearch import breadthFirstSearchWithExplored
from DepthFirstSearch  import depthFirstSearchWithExplored
from AStarWithExplored import aStarWithExplored
from GreedyHeuristicSearch import greedyHeuristicSearch
from scipy.spatial import distance
import time

# Author: Dawson Theroux (SN: 101106602)

def generateGraphFromShapefile(filename):
    print(f"Loading Shapefile...")
    plt.rcParams["figure.figsize"] = (50,20)
    #conda activate OSMNX
    G = nx.read_shp(filename)
    maxX, minX, maxY, minY= None,None,None,None
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

    print(f"Edges before Graph is fixed: {G.number_of_edges()}")
    print(f"Nodes before Graph is fixed: {G.number_of_nodes()}")
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
    print(f"Edges After Graph is fixed: {X.number_of_edges()}")
    print(f"Nodes After Graph is fixed: {X.number_of_nodes()}")
    print(f"Finished creating graph in {time.time() - start_time}")
    return X, pos, minX, maxX, minY, maxY, averageX, averageY 

def plotMap(plotTitle, outputFileLocation, X, path,explored, pos, minX, maxX, minY, maxY):
    # Output file location should include the whole filepath and name
    print("Done adding edges")

    # Add the base map
    nx.draw_networkx_nodes(X,pos,node_size=1,node_color='k')
    nx.draw_networkx_edges(X,pos, width=1, edge_color='k')

    # Add the explored area in red
    nx.draw_networkx_nodes(explored, pos, node_size=3, node_color = "r")
    nx.draw_networkx_edges(explored,pos, width=3, edge_color = "r")

    # Add the actual path.
    nx.draw_networkx_nodes(path, pos, node_size=5, node_color='g')
    nx.draw_networkx_edges(path,pos, width=5, edge_color='g')
    startGraph = nx.Graph()
    startGraph.add_node(list(path)[0])
    nx.draw_networkx_nodes(startGraph,pos, node_size=1, node_color='b')
    print("Done Drawing Edges")
    plt.xlim(minX, maxX) #This changes and is problem specific
    plt.ylim(minY, maxY) #This changes and is problem specific
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title(plotTitle)
    plt.savefig(f"{outputFileLocation}")
    plt.clf()

def generateGraphFromPath(path, nodesList):
    pathGraph = nx.Graph()
    for i,node in enumerate(list(path)):
        pathGraph.add_node(nodesList[node])
        if i + 1 < len(list(path)):
            pathGraph.add_edge(nodesList[node], nodesList[path[i + 1]])
    return pathGraph


def findStartAndEnd(nodesList, averageX, averageY, maxX, maxY, minX, minY, pos):
    startnode = None
    endnode = None
    # Get the node that is closest to 10% in from each side.
    # Get the node with the closest euclidian distance to tenpercentOfGraph and averageY
    tenPercentOfGraph = maxX - minX #The full size of the grpah.
    tenPercentOfGraph = tenPercentOfGraph * 0.1

    closestDistanceToMinIdeal = None
    closestDistanceToMaxIdeal = None
    for i, node in enumerate(nodesList):
        distanceToMinIdeal = distance.euclidean(pos[i], (minX + tenPercentOfGraph, averageY))
        distanceToMaxIdeal = distance.euclidean(pos[i], (maxX - tenPercentOfGraph, averageY))
        if closestDistanceToMinIdeal == None and closestDistanceToMaxIdeal == None:
            closestDistanceToMaxIdeal = distanceToMaxIdeal
            closestDistanceToMinIdeal = distanceToMinIdeal
            startnode = i
            endnode = i
        elif closestDistanceToMaxIdeal > distanceToMaxIdeal:
            closestDistanceToMaxIdeal = distanceToMaxIdeal
            endnode = i
        elif closestDistanceToMinIdeal > distanceToMinIdeal:
            closestDistanceToMinIdeal = distanceToMinIdeal
            startnode = i
            
            
    return startnode, endnode 

def runTests(filename, outputFileLocation):
    graph, pos, minX, maxX, minY, maxY, averageX, averageY= generateGraphFromShapefile(filename)
    nodesList = list(graph.nodes())
    startnode, endnode = findStartAndEnd(nodesList, averageX, averageY, maxX, maxY, minX, minY, pos)

    # Test AStar
    print("Running aStar Euclidean")
    start_time = time.time()
    aStarpathE, aStarExploredGraphE = aStarWithExplored(graph, nodesList[startnode], nodesList[endnode], distance.euclidean, pos)
    aStarTimeE = time.time() - start_time
    aStarpathGraphE = generateGraphFromPath(aStarpathE, nodesList)
    plotMap("A Star Search with Euclidean Distance", f"{outputFileLocation}\\AStarSearchEuclidean.png", graph, aStarpathGraphE, aStarExploredGraphE, pos,minX,maxX,minY,maxY)

    # Test AStar
    print("Running aStar Manhattan")
    start_time = time.time()
    aStarpathM, aStarExploredGraphM = aStarWithExplored(graph, nodesList[startnode], nodesList[endnode], distance.cityblock, pos)
    aStarTimeM = time.time() - start_time
    aStarpathGraphM = generateGraphFromPath(aStarpathM, nodesList)
    plotMap("A Star Search with Manhattan Distance", f"{outputFileLocation}\\AStarSearchManhattan.png", graph, aStarpathGraphM, aStarExploredGraphM, pos,minX,maxX,minY,maxY)

    # Test GHS
    print("Running GHS Euclidean")
    start_time = time.time()
    ghspathE, ghsExploredGraphE = greedyHeuristicSearch(graph, nodesList[startnode], nodesList[endnode], distance.euclidean, pos)
    ghsTimeE = time.time() - start_time
    ghspathGraphE = generateGraphFromPath(ghspathE, nodesList)
    plotMap("Greedy Heuristic Search Euclidean", f"{outputFileLocation}\\GHSEuclidean.png", graph, ghspathGraphE, ghsExploredGraphE, pos,minX,maxX,minY,maxY)

    print("Running GHS Manhattan")
    start_time = time.time()
    ghspathM, ghsExploredGraphM = greedyHeuristicSearch(graph, nodesList[startnode], nodesList[endnode], distance.cityblock, pos)
    ghsTimeM = time.time() - start_time
    ghspathGraphM = generateGraphFromPath(ghspathM, nodesList)
    plotMap("Greedy Heuristic Search Manhattan", f"{outputFileLocation}\\GHSManhattan.png", graph, ghspathGraphM, ghsExploredGraphM, pos,minX,maxX,minY,maxY)

    # Test dijkstras
    print("Running dijkstras")
    start_time = time.time()
    dijkstraspath, dijkstrasExploredGraph = dijkstra_explored(graph, nodesList[startnode], nodesList[endnode], pos)
    dijkstrasTime = time.time() - start_time
    dijkstraspathGraph = generateGraphFromPath(dijkstraspath, nodesList)
    plotMap("Dijkstras Algorithm", f"{outputFileLocation}\\Dijkstras.png", graph, dijkstraspathGraph, dijkstrasExploredGraph, pos,minX,maxX,minY,maxY)

    # Test bfs 
    print("Running BFS")
    start_time = time.time()
    bfspath, bfsExploredGraph = breadthFirstSearchWithExplored(graph, nodesList[startnode], nodesList[endnode], pos)
    bfsTime = time.time() - start_time
    bfspathGraph = generateGraphFromPath(bfspath, nodesList)
    plotMap("Breadth First Search", f"{outputFileLocation}\\BFS.png", graph, bfspathGraph, bfsExploredGraph, pos,minX,maxX,minY,maxY)

    # Test dfs
    print("Running DFS")
    start_time = time.time()
    dfspath, dfsExploredGraph = depthFirstSearchWithExplored(graph, nodesList[startnode], nodesList[endnode], pos)
    dfsTime = time.time() - start_time
    dfspathGraph = generateGraphFromPath(dfspath, nodesList)
    plotMap("Deapth First Search", f"{outputFileLocation}\\DFS.png", graph, dfspathGraph, dfsExploredGraph, pos,minX,maxX,minY,maxY)


    with open(f"{outputFileLocation}\\RunningTimes.txt", "w") as f:
        f.write(f"aStar Time Euclidean: {aStarTimeE}\n")
        f.write(f"aStar Time Manhattan: {aStarTimeM}\n")
        f.write(f"GHS Time Euclidean: {ghsTimeE}\n")
        f.write(f"GHS Time Manhattan: {ghsTimeM}\n")
        f.write(f"Dijkstras Time: {dijkstrasTime}\n")
        f.write(f"BFS Time: {bfsTime}\n")
        f.write(f"DFS Time: {dfsTime}\n")



def main():
    outputFileLocation = "output\\Ottawa\\"
    ottawafilename = "OttawaData/OttawaData.shp"
    runTests(ottawafilename, outputFileLocation)

    ''' 
    outputFileLocation = "output\\Toronto\\"
    torontofilename = "TorontoData/Toronto_Road_Explort.shp"
    runTests(torontofilename, outputFileLocation)
    '''


    '''
    outputFileLocation = "output\\NYC\\"
    nycfilename = "NYCData2/NYCData2.shp"
    runTests(nycfilename, outputFileLocation)
    '''




if __name__ == "__main__":
    main()