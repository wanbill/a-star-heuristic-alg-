from expand import expand
import heapq  # Import heapq to use a priority queue for the open set

def a_star_search(dis_map, time_map, start, end):
    open_set = []  # Use a priority queue (min-heap) instead of a set
    close_set = set()
    
    g_values = {}  # Dictionary to store g values for nodes
    f_values = {}  # Dictionary to store f values for nodes
    track_list = {}  # Dictionary to store parent nodes
    
    g_values[start] = 0
    f_values[start] = dis_map[start][end] #initial distance
    
    # tuple (f_value, node) for the priority queue
    heapq.heappush(open_set, (f_values[start], start))
    
    while open_set:
        _, current = heapq.heappop(open_set)  # Get the node with the smallest f_value
        
        if current == end: #reach the end
            # Reconstruct the path from the end to the start
            path = []
            while current:
                path.append(current)
                current = track_list.get(current)
                
            return path[::-1]  # Reverse the path to get start to end
            
        close_set.add(current)
        next_level = expand(current, time_map)
        
        for neighbor in next_level:
            if neighbor in close_set:
                continue
            
            tentative_g = g_values[current] + time_map[current][neighbor]# calc #the new g val for neighbor
            
            if neighbor not in [node for _, node in open_set]:
                # Neighbor not in open_set, add it with its f_value as the priority
                track_list[neighbor] = current
                g_values[neighbor] = tentative_g
                f_values[neighbor] = g_values[neighbor] + dis_map[neighbor][end] #h_values[neighbor]
                heapq.heappush(open_set, (f_values[neighbor], neighbor))
            elif tentative_g < g_values.get(neighbor):
                # This path to neighbor is better than any previous one. Record it
                track_list[neighbor] = current
                g_values[neighbor] = tentative_g
                f_values[neighbor] = g_values[neighbor] + dis_map[neighbor][end]#h_values[neighbor]
    
    # If no path is found
    return None

                                       
def dfs_helper(time_map, current, end, path, found_path):
    path.append(current)
  

    if current == end:
        found_path.extend(list(path))  # Make a copy of the current path
        return True  # Indicate that we found a path and want to stop the recursion

    next_level = expand(current, time_map)
    for next_node in next_level:# first visit right then left, use reversed 
                                # next_level to first visit left than right
        if next_node not in path:
            if dfs_helper(time_map, next_node, end, path, found_path):
                return True  # Stop the recursion if a path is found

    path.pop()  # Backtrack by removing the last node

def depth_first_search(time_map, start, end):
    if start == end:
        return [start]  # Start and end are the same, return a list containing just start
    found_path = []
    dfs_helper(time_map, start, end, [], found_path)
    
    return found_path

def bfs_helper(time_map, start, end):
    visited = set() #ensure unique node so use set
    queue = [(start, [start])]
    
    while queue: # while queue is not empty
        node, path = queue.pop(0)
        
        if node not in visited: # if the node is not visited--> prevent visited 
                                # same node
            visited.add(node)
            
            if node == end: # found the destination return path
                return path
            
            neighbors = expand(node, time_map)
            
            for neighbor in neighbors: # loop next level
                if neighbor not in visited: # prevent for explore repeated 
                                            #neighbor
                    new_path = path + [neighbor]
                    queue.append((neighbor, new_path))
                    
    return []

def breadth_first_search(time_map, start, end):
    if start == end:
        return [start]   
    shortest_path = bfs_helper(time_map, start, end)
    return shortest_path

# if __name__== "__main__":
#     #path = depth_first_search(time_mapT, 'Alex_Robbinson', 'Aaron_Stone')
#     #print(path)
#     #path2 = depth_first_search(time_mapT, 'Alex_Robbinson', 'Raj_Gupta')
#     #print(path2)
#     #path3 = breadth_first_search(time_map1, 'John_Stevens', 'Mariana_Cardoso')
#     #print(path3)
#     #path4 = breadth_first_search(time_mapM, 'Alex_Robbinson', 'George_Richford')
#     #print(path4)
#     path5 = a_star_search(dis_map5, time_map5, 'John_Stevens', 'Sarah_Parker')
#     print(path5)
 