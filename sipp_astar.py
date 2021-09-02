"""
SIPP implementation  
author: Ashwin Bose (@atb033)
See the article: DOI: 10.1109/ICRA.2011.5980306
"""

from math import fabs
from graph_generation import SippGraph, State
import heapq

class SippPlanner(SippGraph):
    def __init__(self, filename, agent_info, agent_id, dyn_obstacles):
        SippGraph.__init__(self, filename, dyn_obstacles)
        
        self.start = agent_info[2*agent_id]
        self.goal = agent_info[2*agent_id+1]
        self.name = agent_id
        self.open = []
        self.max_path = 0

    def get_successors(self, state):
        successors = []
        m_time = 1
        neighbour_list = self.get_valid_neighbours(state.position)

        for neighbour in neighbour_list:
            start_t = state.time + m_time
            end_t = state.interval[1] + m_time
            for i in self.sipp_graph[neighbour].interval_list:
                if i[0] > end_t or i[1] < start_t:
                    continue
                time = max(start_t, i[0]) 
                s = State(neighbour, time, i)
                successors.append(s)
        return successors

    def get_heuristic(self, position):
        return fabs(position[0] - self.goal[0]) + fabs(position[1]-self.goal[1])

    def get_first(self, element):
        return element[0]

    #compute_plan + get_plan = A-star function
    #compute_plan() determines if plan exists for specific problem and if yes, get_plan() retrieves said plan
    def compute_plan(self):
        self.open = []
        goal_reached = False
        cost = 1
        e_nodes = 0
        g_nodes = 0

        s_start = State(self.start, 0) 

        self.sipp_graph[self.start].g = 0.
        f_start = self.get_heuristic(self.start)
        self.sipp_graph[self.start].f = f_start

        self.open.append((f_start, s_start))

        #following algorithm described in original paper (fig 4)
        while (not goal_reached):
            # print("OPEN LIST:")
            # print(self.open)
            if self.open == []: 
                # Plan not found
                # print("Expected to this msg for no plans found")
                return 0
            s = self.open.pop(0)[1]
            e_nodes += 1
            successors = self.get_successors(s)

            for successor in successors:
                #print("Successor pos:", successor.position)
                #print("successor g(): ", self.sipp_graph[successor.position].g)
                #print("parent g(): ", self.sipp_graph[s.position].g + cost)
                
                if self.sipp_graph[successor.position].g > self.sipp_graph[s.position].g + cost:

                    self.sipp_graph[successor.position].g = self.sipp_graph[s.position].g + cost
                    self.sipp_graph[successor.position].parent_state = s

                    #print("Successor time: ", successor.time)
                    #print("Max path: ", self.max_path)
                    if successor.position == self.goal and successor.time > self.max_path:
                        print("Plan successfully calculated!!")
                        goal_reached = True
                        break

                    self.sipp_graph[successor.position].f = self.sipp_graph[successor.position].g + self.get_heuristic(successor.position)
                    self.open.append((self.sipp_graph[successor.position].f, successor))
                    g_nodes += 1
            
            self.open.sort(key = self.get_first)
            

        # Tracking back
        start_reached = False
        self.plan = []
        current = successor
        while not start_reached:
            self.plan.insert(0,current)
            if current.position == self.start:
                start_reached = True
            current = self.sipp_graph[current.position].parent_state
        print("Expanded Nodes: ", e_nodes)
        print("Generated Nodes: ", g_nodes)
        return 1
            
    def get_plan(self):
        path_list = []

        # first setpoint
        setpoint = self.plan[0]
        temp_dict = {"x":setpoint.position[0], "y":setpoint.position[1], "t":setpoint.time}
        path_list.append(temp_dict)

        for i in range(len(self.plan)-1):
            for j in range(self.plan[i+1].time - self.plan[i].time-1):
                x = self.plan[i].position[0]
                y = self.plan[i].position[1]
                t = self.plan[i].time
                setpoint = self.plan[i]
                temp_dict = {"x":x, "y":y, "t":t+j+1}
                path_list.append(temp_dict)

            setpoint = self.plan[i+1]
            temp_dict = {"x":setpoint.position[0], "y":setpoint.position[1], "t":setpoint.time}
            path_list.append(temp_dict)

        #print("GET_PLAN")
        #print(path_list)
        return path_list



