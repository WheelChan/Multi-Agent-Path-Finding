import time as timer
from sipp_astar import SippPlanner
from graph_generation import SippGraph, State

def process_result(result):
    """" Converts dictionary path to tuple array path for all agents """

    presult = []

    for i in range(len(result)):
        li=[]
        for k in range(len(result[i])):
            li.append((result[i][k]['x'], result[i][k]['y']))
        presult.append(li)            
    return presult

class SIPP_IndependentSolver(object):
    """A planner that plans for each robot independently."""

    def __init__(self, filename, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.filename = filename
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        # self.heuristics = []
        # for goal in self.goals:
        #     self.heuristics.append(get_heuristic(my_map, goal))
    
    

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []

        for i in range(self.num_of_agents):  # Find path for each agent
            print("Inside SIPP start finding a soln!!!!\n")
            sipp_planner = SippPlanner(self.filename,self.my_map.agent_info, i, result)
            if result != []:
                    sipp_planner.max_path = len(result[0])

            if sipp_planner.compute_plan():               
                plan = sipp_planner.get_plan()
                print("!!!! PLAN: ")
                print(plan)
                result.append(plan)
                print(result)
                #TODO add a time constraint for agent to be at goal location at designated time
                
            else:
                raise BaseException('No solutions')

        ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        # print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        #need to process results to be in same format used for individual assignment functions
        presult = process_result(result)

        return presult
