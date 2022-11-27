from queue import PriorityQueue
from math import inf
# graph dependency  
import networkx as nx
from scipy.spatial import distance


# Reference: https://leetcode.com/problems/shortest-path-in-binary-matrix/discuss/313347/A*-search-in-Python#:~:text=An%20A*%20search%20is%20like,estimate%20of%20the%20remaining%20distance.
# Code based on the code from the link above.
def aStarWithExplored(graph, start, end, heuristic, pos):
    def backtrace(prev, start, end):
        node = end
        path = []
        print(f"End node: {end}")
        while node != start:
            path.append(node)
            node = prev[node]
        path.append(node)
        path.reverse()
        return path

    visited = set()
    prev = dict()
    dist = {v: inf for v in list(nx.nodes(graph))} 
    dist[start] = 0
    frontier = PriorityQueue()
    frontier.put((dist[start], start))
    graphList = list(nx.nodes(graph))
    visitedGraph = nx.Graph()
    while frontier.qsize() > 0:
        #print(f" Getting from frontier.")
        curr_cost, curnode = frontier.get()
        if curnode in visited:
            continue
        if curnode == end:
            return backtrace(prev, start, end), visitedGraph
        visited.add(curnode)
        visitedGraph.add_node(graphList[curnode])
        if curnode in prev.keys():
            visitedGraph.add_edge(graphList[curnode], graphList[prev[curnode]])

        for neighbour in dict(graph.adjacency()).get(curnode):   # For all neighbours of the current Node.
            frontier.put((dist[curnode] + distance.euclidean(pos[curnode], pos[neighbour]) + heuristic(pos[neighbour], pos[end]), neighbour))

            pathcost = dist[curnode] + distance.euclidean(pos[curnode], pos[neighbour]) 
            if pathcost < dist[neighbour]:
                dist[neighbour] = pathcost
                prev[neighbour] = curnode

    return backtrace(prev, start, end), visitedGraph