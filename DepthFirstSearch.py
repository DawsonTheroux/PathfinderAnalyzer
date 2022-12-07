from queue import LifoQueue 
from math import inf
import networkx as nx
from scipy.spatial import distance

def depthFirstSearchWithExplored(graph, start, end, pos):
    def findPath(prev, start, end):
        node = end
        path = []
        while node != start:
            path.append(node)
            node = prev[node]
        path.append(node)
        path.reverse()
        return path

    frontier = LifoQueue()
    visitedNodes = []
    prev = {}
    visitedGraph = nx.Graph()
    graphList = list(nx.nodes(graph))
    frontier.put(start)
    visitedNodes.append(start)
    while(frontier.qsize() > 0):
        curNode = frontier.get() 
        visitedGraph.add_node(graphList[curNode])
        if curNode in prev.keys():
            visitedGraph.add_edge(graphList[curNode], graphList[prev[curNode]])

        if curNode == end:
            break

        for neighbour in dict(graph.adjacency()).get(curNode):   # For all neighbours of the current Node.
            if neighbour not in visitedNodes:
                frontier.put(neighbour) 
                visitedNodes.append(neighbour)
                prev[neighbour] = curNode




    # Path is a list of node indecies.
    # Visited Graph is a constructed NetworkX graph.
    return findPath(prev, start, end), visitedGraph