"""
Python implementation of Conflict-based search
author: Ashwin Bose (@atb033)
"""
import sys
sys.path.insert(0, '../')
import argparse
from math import fabs
from itertools import combinations
from copy import deepcopy
import time as timer
from sipp_astar import SippPlanner
from graph_generation import SippGraph, State
import random
import time as timer


class HighLevelNode(object):
    def __init__(self):
        self.solution = []
        self.constraint_dict = []
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost

class SIPP_CBSSolver(object):
    #TODO: bool variable - collision constraint to indicate interval allow
    #TODO: add bool variable to plan in find soln
    #TODO: create find_soln()
    #TODO: parse path to find neg paths to add as dynamic obstacles
    #TODO: create_constraint_from_conflict() - change vertex and edge constraint -> time interval constraint ; +/- 1 timestep of safe interval
    #TODO: update edge & vertex collision -> dynamic obstacles 
    #TODO: lovel search - astar

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

        self.open_set = set()
        self.closed_set = set()
    
    # finds actual collision and returns path
    def detect_collision(self,path1, path2):    
        max_t = max(len(path1), len(path2))
        #print("max t")
        #print(max_t)

        if max_t == len(path1):
            longer = 1
        else:
            longer = 2
        
        # list of obstacles
        # may need to consider edge collision
        for t in range(max_t):
            if t >= len(path1):
                loc_1 =(path1[len(path1)-1]['x'], path1[len(path1)-1]['y'])
            else:    
                loc_1 =(path1[t]['x'], path1[t]['y'])

            if t >= len(path2):
                loc_2 =(path2[len(path2)-1]['x'], path2[len(path2)-1]['y'])
            else:    
                loc_2 =(path2[t]['x'], path2[t]['y'])

            #for vertice collisions
            if loc_1 == loc_2:
                if longer == 1:
                    return {'typeEdge': False, 'path':path1[t]}
                else:
                    return {'typeEdge': False, 'path':path2[t]}

            #for edge collisions
            if t == 0:
                prev_loc1 = loc_1
                prev_loc2 = loc_2
            else:
                #print(prev_loc1, prev_loc2, loc_1, loc_2)
                if loc_2 == prev_loc1 and loc_1 == prev_loc2:
                    #randomly choose agent and then build dynamic obstacle based on the path
                    #using disjoint splitting so can randomly choose aggent to build constraint for now instead of later
                    randvalue = random.randint(0,1)
                    if randvalue == 0:
                        return {'typeEdge': True, 'agent': 1, 'path': path1[t]}
                    else:
                        return {'typeEdge': True, 'agent': 2, 'path': path2[t]}
                else:
                    prev_loc1 = loc_1
                    prev_loc2 = loc_2
    
    # iterates through agents to find collisions
    def agent_get_first_collision(self, paths):
        result = []
        index = 0
        for i in range(len(paths)):
            for k in range(i+1, len(paths)):
                tmp = self.detect_collision(paths[i], paths[k])
                if tmp is None:
                    continue
                if tmp['typeEdge'] == True:
                    if tmp['agent'] == 1:
                        curr_agent = i
                    else:
                        curr_agent = k 
                    result = {'agent1': curr_agent, 'agent2': curr_agent, 'collision_loc': tmp['path'] }
                else:
                    result = {'agent1': i, 'agent2': k, 'collision_loc': tmp['path'] }
                
                #collision detected
                if result['collision_loc'] != None:
                    return result
        # no collision found
        return None
    
    #x,y,t (path) == dynamic obstacles
    def add_constraint(self, constraint):
        constraint_set = []
        for loc in constraint: # same loc diff ts
            #print("Add_constraint(): loc - ")
            #print(loc)
            constraint_set.append( {'x':loc['x'], 'y':loc['y'], 't':loc['t'] } )
        return constraint_set

    # disjoint splitting
    # TODO: potential edge constraints
    # used negative value for chosen agent to build positive constraint for specific agent (all other agents cannot be in location)
    # dynamic obstacle built for negative constraint agents if agent != agent * -1 (seen in line 239)
    # all agent values are +1 of original because of agent 0 edge case, 0 * -1 =0 and so if agent 0 is chosen no constraints ever built for other agents
    ## as such, all agent values are +1 of what they actually are and so checks need to be agent - 1 in order to find true agent value
    def create_constraints_from_conflict(self, conflict):
        value = random.randint(0,1)
        if value == 0:
            chosen_agent = conflict['agent1']+1
        else:
            chosen_agent = conflict['agent2']+1

        constraint_dict = []

        temp = []
        for i in range(-1, 2): #two sets of constraint
            #print("\tCONFLICT:")
            #print(conflict)
            if conflict['collision_loc']['t'] == 0:
                continue
            temp.append({'agent': chosen_agent, 'x':conflict['collision_loc']['x'], 'y': conflict['collision_loc']['y'], 't': conflict['collision_loc']['t']+i}) #choosen agent can't be in this loc at ts +/- 1 -> negative constraint
        constraint_dict.append(temp)
        
        temp = []
        for i in range(-1, 2): #two sets of constraint
            if conflict['collision_loc']['t'] == 0:
                continue
            temp.append({'agent': -1 * chosen_agent, 'x':conflict['collision_loc']['x'], 'y': conflict['collision_loc']['y'], 't': conflict['collision_loc']['t']+i}) #applies to every other agent can't be in this loc at ts +/- 1 -> positive constraint i.e. dynamical obstacle for other agents
        constraint_dict.append(temp)
        return constraint_dict

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution])

    def process_result(self, result):
        """" Converts dictionary path to tuple array path for all agents """

        presult = []

        for i in range(len(result)):
            li=[]
            for k in range(len(result[i])):
                li.append((result[i][k]['x'], result[i][k]['y']))
            presult.append(li)            
        return presult    

    def find_solution(self):
        start_time = timer.time()
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        # TODO: low level search for each agent with SIPP
        e_nodes = 0
        g_nodes = 0
        
        start.constraint_dict = {}
        #low level search
        for agent in range(self.num_of_agents):
            start.constraint_dict[agent] = []
            sipp_planner = SippPlanner(self.filename, self.my_map.agent_info, agent, start.constraint_dict[agent])
            if sipp_planner.compute_plan():
                start.solution.append(sipp_planner.get_plan())
        print("Start.Solution")
        #print(start.solution)
        start.cost = self.compute_solution_cost(start.solution)             

        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            e_nodes += 1
            self.open_set -= {P}
            self.closed_set |= {P}

            conflict_dict = self.agent_get_first_collision(P.solution)

            #found solution
            if not conflict_dict:
                print("solution found")
                print("CBS g_nodes: ", g_nodes)
                print("CBS e_nodes: ", e_nodes)
                result = self.process_result(self.generate_plan(P.solution))
                self.CPU_time = timer.time() - start_time
                return result

            tmp_constraint_dict = self.create_constraints_from_conflict(conflict_dict)

            #creating constraints
            for constraint in tmp_constraint_dict:
                #print("Cur Constraint: " )
                #print(constraint)
                emptyPathFound = False
                new_node = deepcopy(P)
                if constraint[0]['agent']-1 >= 0:
                    curr_agent = constraint[0]['agent']-1
                    new_node.constraint_dict[curr_agent].append(self.add_constraint(constraint))

                else:
                    tmp = self.add_constraint(constraint)
                    for agent in range(self.num_of_agents):
                        if agent == constraint[0]['agent']*-1-1:
                            continue
                        
                        curr_agent = agent
                        new_node.constraint_dict[agent].append(tmp)
                        
                for agent in range(self.num_of_agents):
                    sipp_planner = SippPlanner(self.filename, self.my_map.agent_info, agent, new_node.constraint_dict[agent])
                    # print("SIPP PLANNER_cp:")
                    # print(sipp_planner.compute_plan())
                    if sipp_planner.compute_plan():
                        new_node.solution[agent] = sipp_planner.get_plan()
                        #print("New node solution")
                        #print(new_node.solution[agent])
                    else:
                        #print("Path failed for: ", agent) 
                        emptyPathFound = True 
                
                if emptyPathFound:
                    continue
                new_node.cost = self.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}
                    g_nodes += 1
                    #print("Added new node")
        print("Solution NOT found")
        return {}

    def generate_plan(self, solution):
        plan = []
        for agent_path in solution:
            path_dict_list=[]
            for loc in agent_path:
                path_dict_list.append( {'x':loc['x'], 'y':loc['y'], 't':loc['t']})
            plan.append(path_dict_list)

        return plan

