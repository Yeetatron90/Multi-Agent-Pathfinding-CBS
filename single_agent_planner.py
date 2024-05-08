import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    constraint_table = {}
    # Iterate through each constraint in the list of constraints
    for constraint in constraints:
        # Check if the constraint applies to the agent in question
        if constraint['agent'] == agent:
            timestep = constraint['timestep']
            # Constraint without a specified timestep applies to all timesteps
            if timestep not in constraint_table:
                constraint_table[timestep] = []
            constraint_table[timestep].append(constraint['loc'])
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


'''
def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    # Check for any constraints that apply to the next move at the given timestep
    if next_time in constraint_table:
        for loc in constraint_table[next_time]:  # Directly iterate through constrained locations
            if loc == [next_loc]:
                return True  # The move to next_loc is constrained
    return False  # No constraints were violated
'''



def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    # Check if there are any constraints for the given timestep
    #import ipdb
    #ipdb.set_trace()
    if next_time in constraint_table:
        # Iterate through all constraints at this time step
        for constraint_loc in constraint_table[next_time]:
            # Handling edge constraints (defined by a pair of locations)
            if len(constraint_loc) == 2:
                # An edge constraint is satisfied if both the current and next locations match the constraint
                if constraint_loc == [curr_loc, next_loc]:
                    return True  # This move is explicitly prohibited by an edge constraint
            # Handling single location constraints
            elif len(constraint_loc) == 1:
                # A single location constraint is satisfied if the next location matches the constraint
                if constraint_loc == [next_loc]:
                    return True  # Moving to next_loc is prohibited
    return False  # No constraints were violated for this move at this timestep




def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']



def is_within_bounds(loc, my_map):
    rows, cols = len(my_map), len(my_map[0])
    return 0 <= loc[0] < rows and 0 <= loc[1] < cols


def reach_des_constrained(goal_loc, current_time_step, constraint_table, max_future_check=100):

    for future_time in range(current_time_step + 1, current_time_step + max_future_check):
        if future_time in constraint_table:
            for constraint in constraint_table[future_time]:
                if len(constraint) == 1 and constraint == goal_loc:
                    # Found a vertex constraint that directly affects the goal location
                    return False
                elif len(constraint) == 2 and goal_loc in constraint:
                    # Found an edge constraint that affects the goal location
                    return False
    return True




def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):


    constraint_table = build_constraint_table(constraints, agent)

    open_list = []
    closed_list = {}
    root = {
        'loc': start_loc,
        'g_val': 0,
        'h_val': h_values[start_loc],
        'parent': None,
        'time_step': 0
    }

    # Initially push the root node into the open list
    push_node(open_list, root)
    # Mark the root node as visited in the closed list
    closed_list[(root['loc'], root['time_step'])] = root

    while open_list:
        # Pop the node with the lowest f_val from the open list
        curr = pop_node(open_list)

        # Check if the current node is the goal and not constrained
        if curr['loc'] == goal_loc:
            if reach_des_constrained(goal_loc, curr['time_step'], constraint_table):
                return get_path(curr)

        # Explore adjacent nodes
        for dir in range(4):
            child_loc = move(curr['loc'], dir)
            # Skip if the location is an obstacle or if moving to this location is constrained
            if not is_within_bounds(child_loc, my_map) or my_map[child_loc[0]][child_loc[1]] or is_constrained(curr['loc'], child_loc, curr['time_step'] + 1, constraint_table):
                continue

            # Construct the child node
            child = {
                'loc': child_loc,
                'g_val': curr['g_val'] + 1,
                'h_val': h_values[child_loc],
                'parent': curr,
                'time_step': curr['time_step'] + 1
            }

            # If this child location at this timestep has not been visited or offers a better path, update the lists
            if (child['loc'], child['time_step']) not in closed_list:
                push_node(open_list, child)
                closed_list[(child['loc'], child['time_step'])] = child

        # Consider waiting at the current location
        wait_child = {
            'loc': curr['loc'],
            'g_val': curr['g_val']+1,
            'h_val': h_values[curr['loc']],
            'parent': curr,
            'time_step': curr['time_step']+1
        }

        # Add the waiting action if it does not violate any constraints
        if not is_constrained(curr['loc'], wait_child['loc'], wait_child['time_step'], constraint_table):
            if (wait_child['loc'], wait_child['time_step']) not in closed_list:
                push_node(open_list, wait_child)
                closed_list[(wait_child['loc'], wait_child['time_step'])] = wait_child

    # Return None if no path to the goal is found
    return None

