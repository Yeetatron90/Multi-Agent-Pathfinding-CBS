import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches            
            '''''
            for t in range(len(path)):
                if t < len(path) - 1:
                    curr_loc = path[t]
                    next_loc = path[t + 1]
                    for future_agent in range(i + 1, self.num_of_agents):
                        constraints.append({'agent': future_agent, 'loc': [curr_loc, next_loc], 'timestep': t})
                # Add vertex constraint for the current location for future agents
                for future_agent in range(i + 1, self.num_of_agents):
                    constraints.append({'agent': future_agent, 'loc': [path[t]], 'timestep': t})
            '''''
            
            for next_agent in range(self.num_of_agents):

                
                for timesteps in range(len(path)):
                    if next_agent != i:
                        constraints.append({'agent' : next_agent, 'loc' : [path[timesteps]], 'timestep' : timesteps})
                        if timesteps > 0:
                            constraints.append({'agent' : next_agent, 'loc' : [path[timesteps],path[timesteps-1]], 'timestep' : timesteps})  
                for i in range (0,20):
                    constraints.append({'agent' : next_agent, 'loc' : [path[len(path) - 1]], 'timestep' : len(path)+i - 1})     

                    

            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
