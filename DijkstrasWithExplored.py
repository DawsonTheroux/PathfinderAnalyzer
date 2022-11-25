# dependencies for our dijkstra's implementation
from queue import PriorityQueue
from math import inf
# graph dependency  
import networkx as nx
from scipy.spatial import distance


"""Dijkstra's shortest path algorithm"""
def dijkstra_explored(graph: 'networkx.classes.graph.Graph', start: str, end: str, pos) -> 'List':
    """Get the shortest path of nodes by going backwards through prev list
    credits: https://github.com/blkrt/dijkstra-python/blob/3dfeaa789e013567cd1d55c9a4db659309dea7a5/dijkstra.py#L5-L10"""
    def backtrace(prev, start, end):
        node = end
        path = []
        while node != start:
            #print(node)
            path.append(node)
            node = prev[node]
        path.append(node) 
        path.reverse()
        return path
        
    """get the cost of edges from node -> node
    cost(u,v) = edge_weight(u,v)"""
    def cost(u, v):
        #print(f"graph.get_edge_data(u,v) {list(nx.nodes(graph))[u].get.all_attributes()}") 
        return distance.euclidean(pos[u], pos[v])
        
    """main algorithm"""
    # predecessor of current node on shortest path 
    prev = {} 
    # initialize distances from start -> given node i.e. dist[node] = dist(start, node)
    dist = {v: inf for v in list(nx.nodes(graph))} 
    # nodes we've visited
    visited = set() 
    # prioritize nodes from start -> node with the shortest distance!
    ## elements stored as tuples (distance, node) 
    pq = PriorityQueue()  
    visitedGraph = nx.Graph()
    graphList = list(nx.nodes(graph))
    dist[start] = 0  # dist from start -> start is zero
    pq.put((dist[start], start))
    
    while 0 != pq.qsize():
        curr_cost, curr = pq.get()
        visited.add(curr)
        visitedGraph.add_node(graphList[curr])
        if curr in prev.keys():
            visitedGraph.add_edge(graphList[curr], graphList[prev[curr]])

        if curr == end:
            break

        #print(f'visiting {curr}')
        # look at curr's adjacent nodes
        for neighbor in dict(graph.adjacency()).get(curr):
            # if we found a shorter path 
            path = dist[curr] + cost(curr, neighbor)
            if path < dist[neighbor]:
                # update the distance, we found a shorter one!
                dist[neighbor] = path
                # update the previous node to be prev on new shortest path
                prev[neighbor] = curr
                # if we haven't visited the neighbor
                if neighbor not in visited:
                    # insert into priority queue and mark as visited
                    visited.add(neighbor)
                    pq.put((dist[neighbor],neighbor))
                # otherwise update the entry in the priority queue
                else:
                    # remove old
                    _ = pq.get((dist[neighbor],neighbor))
                    # insert new
                    pq.put((dist[neighbor],neighbor))
    print("=== Dijkstra's Algo Output ===")
    print("Distances")
    #print(dist)
    print("Visited")
    print(visited)
    print("Previous")
    print(prev)
    # we are done after every possible path has been checked 
    return backtrace(prev, start, end), dist[end], visitedGraph