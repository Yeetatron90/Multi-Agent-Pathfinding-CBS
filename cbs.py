import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import copy



def get_location(path, t):
    """Helper function to get the location of an agent at time t along its path."""
    return path[t] if t < len(path) else path[-1]

def detect_collision(path1, path2):
    """Detects the first collision between two paths."""
    max_length = max(len(path1), len(path2))
    for t in range(max_length):
        location1 = get_location(path1, t)
        location2 = get_location(path2, t)
        
        # Check for vertex collision
        if location1 == location2:
            return {'type': 'vertex', 'location': [location1], 'timestep': t}
        
        # Check for edge collision (considering the case where one path might be longer)
        if t+1 < max_length:
            next_location1 = get_location(path1, t + 1)
            next_location2 = get_location(path2, t + 1)
            if next_location1 == location2 and next_location2 == location1:
                return {'type': 'edge', 'location': [location1, location2], 'timestep': t + 1}
    
    return None  # No collision detected




def detect_collisions(paths):
    """Detect the first collision between all pairs of paths."""
    collisions = []
    for i in range(len(paths)):
        for j in range(i, len(paths)):
            if i == j:
                continue
            collision = detect_collision(paths[i], paths[j])
            if collision:
                collisions.append({
                    'a1': i,
                    'a2': j,
                    'loc': collision['location'],
                    'timestep': collision['timestep'],
                    'type': collision['type']
                })
    return collisions





def standard_splitting(collision):
    """Generates constraints to resolve the given collision."""
    constraints = []
    if collision['type'] == 'vertex':
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']})
    elif collision['type'] == 'edge':
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']})
    return constraints









class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
            
        while self.open_list:
            current_node = self.pop_node()

            if not current_node['collisions']:
                self.print_results(current_node)
                return current_node['paths']

            collision = current_node['collisions'][0]  # Choose the first collision to resolve
            constraints = standard_splitting(collision)  # Generate new constraints

            for constraint in constraints:
                new_node = copy.deepcopy(current_node)  # Deep copy to avoid reference issues
                new_node['constraints'].append(constraint)

                # Recompute path for the affected agent
                agent_id = constraint['agent']
                new_path = a_star(self.my_map, self.starts[agent_id], self.goals[agent_id],
                                self.heuristics[agent_id], agent_id, new_node['constraints'])

                if new_path is None:  # Path not found with the new constraint, skip this node
                    continue

                # Update the path for the agent and recalculate costs and collisions
                new_node['paths'][agent_id] = new_path
                new_node['cost'] = get_sum_of_cost(new_node['paths'])
                new_node['collisions'] = detect_collisions(new_node['paths'])
                self.push_node(new_node)


        return None


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
