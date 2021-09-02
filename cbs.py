import time as timer
import heapq
import random
import copy
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    #           Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.

    larger = 1
    found = 0
    min_length = min(len(path1), len(path2))
    max_length = max(len(path1), len(path2))
    if len(path1) < len(path2):                     #so that shorter path is checked for longer path
        larger = 2
    for i in range(1, max_length):
        if i == 1:
            prev_cell_a1 = get_location(path1, 0)
            prev_cell_a2 = get_location(path2, 0)
        if larger == 1:                             #if path1 is shorter path
            cell_a1 = get_location(path1, i)
            if i >= min_length:                     #if at end of shorter path already
                cell_a2 = get_location(path2, min_length-1)
            else:
                cell_a2 = get_location(path2, i)
        else:                                       #if path2 is shorter path
            cell_a2 = get_location(path2, i)
            if i >= min_length:                     #if at end of shorter path
                cell_a1 = get_location(path1, min_length-1)
            else:
                cell_a1 = get_location(path1, i)
        if cell_a1 == cell_a2:                      #check current location (vertice collision)
            found = 1
            temp = {'location': cell_a1, 'timestep': i}
        if (prev_cell_a1, cell_a1) == (cell_a2, prev_cell_a2):          #check edge collision
            found = 1
            temp = {'location': (prev_cell_a1, cell_a1), 'timestep': i}
        if found == 1:              #return if collision found
            return temp
        prev_cell_a1 = cell_a1      #update "prev" locations
        prev_cell_a2 = cell_a2
    return None


def detect_collisions(paths):
    ##############################
    #           Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.

    collisions = dict()
    result = []
    index = 0
    for i in range(len(paths)):
        for k in range(i+1, len(paths)):
            result = detect_collision(paths[i], paths[k])
            if result != None:
                temp = {'a1': i, 'a2': k, 'loc': [result['location']], 'timestep': result['timestep']}
                collisions = temp
    return collisions


def standard_splitting(collision):
    ##############################
    #           Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    result = []
    
    if isinstance(collision['loc'][0][0], int):         #this condition determines if vertice (condition = True) or edge (condition = False)
        temp = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        result += [temp]
        temp = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        result += [temp]

    else:           #edge constraints                            
        temp = {'agent': collision['a1'], 'loc': [collision['loc'][0][0], collision['loc'][0][1]], 'timestep': collision['timestep']}
        result += [temp]
        temp = {'agent': collision['a2'], 'loc': [collision['loc'][0][1], collision['loc'][0][0]], 'timestep': collision['timestep']}
        result += [temp]
    return result

def remove_loc_loop(collision):                     #removes repeated vertice collisions (creates constraint that agent cannot move into same location for next timestep)
    result = []
    if isinstance(collision['loc'][0][0], int):
        if len(collision) == 4:                 #for disjoint splitting
            temp = {'agent': collision['agent'] * -2, 'positive': collision['positive'], 'loc': [collision['loc'][0], collision['loc'][0]], 'timestep': collision['timestep']}
        else:                                   #for standard splitting
            temp = {'agent': collision['agent'] * -2, 'loc': [collision['loc'][0], collision['loc'][0]], 'timestep': collision['timestep']}
        result = [temp]
    return result


def disjoint_splitting(collision):
    ##############################
    #           Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    
    result = []
    value = random.randint(0,1)
    if value == 0:
        chosen_agent = collision['a1']
    else:
        chosen_agent = collision['a2']

    #if collision detected is a single tuple (a vertex collision) then the condition in the if statement below
    #will be true and vertex constraints will be creaeted, otherwise edge constraints will be created
    if isinstance(collision['loc'][0][0], int):
        temp = {'agent': chosen_agent, 'positive': True, 'loc': collision['loc'], 'timestep': collision['timestep']}
        result += [temp]
        temp = {'agent': chosen_agent, 'positive': False, 'loc': collision['loc'], 'timestep': collision['timestep']}
        result += [temp]

    else:
        temp = {'agent': chosen_agent, 'positive': True, 'loc': [collision['loc'][0][0], collision['loc'][0][1]], 'timestep': collision['timestep']}
        result += [temp]
        temp = {'agent': chosen_agent, 'positive': False, 'loc': [collision['loc'][0][0], collision['loc'][0][1]], 'timestep': collision['timestep']}
        result += [temp]
    return result


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
        root['collisions'] = [detect_collisions(root['paths'])]
        if not root['collisions'][0]:
            self.print_results(root)
            return root['paths']
        self.push_node(root)

        # Testing
        print(root['collisions'])

        # Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        #           High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        temp = []
        while len(self.open_list) != 0:           #len(self.open_list)
            curr_node = self.pop_node()
            if not curr_node['collisions'][0]:
                self.print_results(curr_node)
                return curr_node['paths']
            #print("ORIGINAL COLLISION: ", curr_node['collisions'])
            curr_collision = curr_node['collisions'][0]
            #curr_constraints = standard_splitting(curr_collision)              #option for performing CBS without disjoint splitting
            curr_constraints = disjoint_splitting(curr_collision)
            #print("BOTH CONSTRAINTS: ", curr_constraints)
            for k in curr_constraints:
                Q = dict()
                temp_constraints = []
                temp_constraints += curr_node['constraints']
                if k not in temp_constraints:                                   #repeated constraints are removed but first, remove edge that 
                    temp_constraints += [k]                                     #moves agent into same location if that is a collision path
                else:
                    temp = remove_loc_loop(k)
                    if temp and temp not in temp_constraints:
                        temp_constraints += temp

                Q['constraints'] = temp_constraints
                #print("CONSTRAINTS: ", Q['constraints'])
                Q['paths'] = curr_node['paths']
                Q_agent = k['agent']
                path = a_star(self.my_map, self.starts[Q_agent], self.goals[Q_agent], self.heuristics[Q_agent],
                              Q_agent, Q['constraints'])
        
                if path is not None:
                    Q['paths'][Q_agent] = path
                    Q['collisions'] = [detect_collisions(Q['paths'])]
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    #print("AT THE END: ", Q['paths'])
                    #print("COLLISIONS: ", Q['collisions'])
                    #'''
                    if not Q['collisions'][0]:                                  #if no collisions, this is our answer
                        self.print_results(Q)
                        print("Path Result: ", Q['paths'])
                        return Q['paths']
                    #'''
                    self.push_node(Q)
                    #print("\n")
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
