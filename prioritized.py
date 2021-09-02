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

        max_time = 0;
        start_time = timer.time()
        result = []
        constraints = []
        goal_points = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            #         Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            t = 0

            #Setting up Negative Constraints
            #check if new path has longer time requirement, max_time is lowest negative number
            temp = len(path) * -1;
            if (temp < max_time):
                old_max_time = max_time;
                max_time = temp;
                
            #set up negative vertice constraints
            for k in path:
                constraints += [{'agent': max_time, 'loc': [k], 'timestep': t}]
                t += 1
                if t == temp * -1:
                    goal_points += [{'agent': max_time, 'loc': [k], 'timestep': t-1}]

            #adding constraints so that agents in goal locations are also constraints for future paths
            times = len(self.my_map) *(temp - old_max_time)
            while times < 0:
                for l in range(len(goal_points)):
                    goal_points[l]['timestep'] += 1
                    constraints += [{'agent': max_time, 'loc': goal_points[l]['loc'], 'timestep': goal_points[l]['timestep']}]
                times += 1

            #set up negative edge constraints using latest path
            t2 = 1
            for j in range(t-1):
                constraints += [{'agent': max_time - 1, 'loc': [path[j], path[j+1]], 'timestep': t2}]
                t2 += 1
            update_ends = False;
            ##############################
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
