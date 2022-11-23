import networkx as nx
import matplotlib.pyplot as plt

def generateGraphFromShapefile(filename):
    plt.rcParams["figure.figsize"] = (50,20)
    #conda activate OSMNX
    G = nx.read_shp(filename)
    maxX, minX, maxY, minY = None,None,None,None

    nodesList = list(G.nodes())
    for i in range(len(nodesList)):
        if maxX == None or maxX < nodesList[i][0]:
            maxX = nodesList[i][0]
        if maxY == None or maxY < nodesList[i][1]:
            maxY = nodesList[i][1]
        if minX == None or minX > nodesList[i][0]:
            minX = nodesList[i][0]
        if minY == None or minY > nodesList[i][1]:
            minY = nodesList[i][1]

    print(f"Edges: {G.number_of_edges()}")
    print(f"Nodes: {G.number_of_nodes()}")
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
    return X, pos, minX, maxX, minY, maxY

def plotMap(X, path, pos, minX, maxX, minY, maxY):
    print("Done adding edges")
    nx.draw_networkx_nodes(X,pos,node_size=1,node_color='r')
    nx.draw_networkx_edges(X,pos, width=1, edge_color='r')
    nx.draw_networkx_nodes(path, pos, node_size=1, node_color='g')
    nx.draw_networkx_edges(path,pos, width=1, edge_color='g')
    print("Done Drawing Edges")
    plt.xlim(minX, maxX) #This changes and is problem specific
    plt.ylim(minY, maxY) #This changes and is problem specific
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('From shapefiles to NetworkX')
    plt.show()

def getShortestPathGraph(graph):
    nodesList = list(graph.nodes())
    path = nx.dijkstra_path(graph,nodesList[50], nodesList[8740])
    print(f"path: {path}")
    pathGraph = nx.Graph()
    for i,node in enumerate(list(path)):
        pathGraph.add_node(nodesList[node])
        if i + 1 < len(list(path)):
            pathGraph.add_edge(nodesList[node], nodesList[list(path)[i + 1]])
    return pathGraph

def main():
    filename = "Data3/Smaller_Roads_V3.shp"
    graph, pos, minX, maxX, minY, maxY = generateGraphFromShapefile(filename)
    shortestPath = getShortestPathGraph(graph)
    plotMap(graph, shortestPath, pos,minX,maxX,minY,maxY)

if __name__ == "__main__":
    main()