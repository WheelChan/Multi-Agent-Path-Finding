import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]                      #added (0,0) for when agent doesn't move
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

def find_max_time(constraints):             #determines longest path created by prev agents, this data kept in agents as a negative number
    max_val = 0                             #the agent with most negative value for 'agent' is the current longest path 
    for i in constraints:
        if i['agent'] < max_val: 
            max_val = i['agent']
    if max_val == 0:
        return -1
    return max_val + 1

def build_disjoint_con_table(constraints, agent):
    #builds table for disjoint constraints, checks "positive" value to see if positive or negative constraint
    #constraints are indexed as (timestep, location, positive/negative constraint) where True = positive constraint and False = negative constraint
    table = dict()
    for i in constraints:
        is_edge = True
        if isinstance(i['loc'][0][0], int):
            is_edge = False
        if i['agent'] == agent and i['positive'] == False:                              #negative constraint for current agent
            if is_edge == True:
                table[(i['timestep'], (i['loc'][0], i['loc'][1]), False)] = i
            else:
                table[(i['timestep'], i['loc'][0], False)] = i
        elif i['agent'] is not agent and i['positive'] == True:                         #positive constraints of other agents
            if is_edge == True:
                table[(i['timestep'], (i['loc'][0], i['loc'][1]), False)] = i
            else:
                table[(i['timestep'], i['loc'][0], False)] = i
        elif i['agent'] == agent and i['positive'] == True:                             #positive constraint for current agent
            if is_edge == True:
                table[(i['timestep'], (i['loc'][0], i['loc'][1]), True)] = i
            else:
                table[(i['timestep'], i['loc'][0], True)] = i
        elif i['agent'] < 0:                                                            #repeated negative constraints
            if is_edge == True:
                table[(i['timestep'], (i['loc'][0], i['loc'][1]), False)] = i
            else:
                table[(i['timestep'], i['loc'][0], False)] = i
            
    return table
           

def build_constraint_table(constraints, agent):
    ##############################
    #               Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    table = dict()
    for i in constraints:                                                               #only add constraint if entry in "constraints" refer to current agent
        if i['agent'] == agent:
            table[i['timestep']] = i
    return table

def build_vpath_constraint_table(constraints, value):                                   #for vertice constraints from agents with higher priority
    p_table = dict()
    for i in constraints:
        if i['agent'] == value:
            temp = {'timestep': i['timestep'], 'loc': (i['loc'][0])}
            p_table[(i['timestep'], temp['loc'])] = temp   
    return p_table

def build_epath_constraint_table(constraints, value):                                   #for edge constraints from agents with higher priority
    e_table = dict()
    for i in constraints:
        if i['agent'] == value and len(i['loc']) == 2:
            temp = {'timestep': i['timestep'], 'loc': (i['loc'])}
            e_table[(temp['loc'][0], temp['loc'][1])] = temp
    return e_table


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


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    #               Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    if next_time in constraint_table:
        if curr_loc == next_loc:
            return constraint_table[next_time]['loc'] == next_loc or constraint_table[next_time]['loc'] == [next_loc, next_loc]
        if curr_loc != next_loc:
            if constraint_table[next_time]['loc'] == [curr_loc, next_loc]:
                return True
            if constraint_table[next_time]['loc'] == [next_loc, curr_loc]:
                return True
    return False

def path_constrained(curr_loc, next_time, constraint_table):                        #check vertice constraints from prev paths
    if (next_time, curr_loc) in constraint_table:
        return True
    return False

def edge_constrained(curr_loc, next_loc, next_time, constraint_table):              #check edge constraints from prev paths
    if (curr_loc, next_loc) in constraint_table:
        if constraint_table[(curr_loc, next_loc)]['timestep'] == next_time:
            return True
    if (next_loc, curr_loc) in constraint_table:
        if constraint_table[(next_loc, curr_loc)]['timestep'] == next_time:
            return True
    return False

def disjoint_constrained(curr_loc, next_loc, next_time, constraint_table):          #for constraints created by disjoint splitting in CBS
    if (next_time,(curr_loc, next_loc), True) in constraint_table:
        return False
    if (next_time, next_loc, True) in constraint_table:
        return False
    if (next_time,(curr_loc, next_loc), False) in constraint_table:
        return True
    if (next_time, next_loc, False) in constraint_table:
        return True
    return False
        

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """


    ##############################
    #           Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    max_time = find_max_time(constraints)                                                                   #keep track of max_time required, value kept track of with negative agent values                                                              
    disjoint = 0            #if current function is disjoint CBS, this value is changed to 1 later on
    if constraints:
        if len(constraints[0]) == 4:                                                                            #for disjoint constraints in CBS, only constraints with 'positive' dictionary value have 4 entries
            disjoint = 1
            constraint_table = build_disjoint_con_table(constraints, agent)
        else:
            constraint_table = build_constraint_table(constraints, agent)                                       #otherwise, perform as usual
    
        #Negative agent constraints for vertices and edges get their own table (negative agent constraints affect all future paths)
        #Negative agent constraints prevent future agents from intersecting with paths created by higher priority agents
        #Use max_time to differentiate between vertices and edges --> agent == max_time is vertice, agent == max_time -1 == edge

            ver_path_constraint_table = build_vpath_constraint_table(constraints, max_time)
            edge_path_constraint_table = build_epath_constraint_table(constraints, max_time - 1)

    else:       #for when constraints is empty (first run for root)
        constraint_table = build_constraint_table(constraints, agent)
        ver_path_constraint_table = build_vpath_constraint_table(constraints, max_time)
        edge_path_constraint_table = build_epath_constraint_table(constraints, max_time - 1)

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}              #added "timestep" to root
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root                                                 #changed closed list to use tuples of (cell,timestep)   
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        #          Adjust the goal test condition to handle goal constraints

        if curr['loc'] == goal_loc and curr['timestep'] > (len(my_map)/2 + 3): #goal constraint adjusted to take into account max_time requirement
            return get_path(curr)                           #(len(my_map)/2 + agent)
        if curr['timestep'] > (3*(len(my_map)/2 + 3)):      #max time allowance is 3 times the goal time requirement
            return None
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if disjoint == 1:                                                                                #if disjoint constraints, we use seperate function for checking disjoint constraint table
                if disjoint_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
                    continue
            else:
                if is_constrained([(child_loc[0],child_loc[1])], [(child_loc[0],child_loc[1])],             #for vertice constraints, is vertice constraint check if variables for start_loc = end_loc
                                   curr['timestep']+1, constraint_table) or is_constrained(curr['loc'],     #for edge constraints, is edge constraint check if variables for start_loc != end_loc
                                    (child_loc[0],child_loc[1]), curr['timestep']+1, constraint_table):     #if constraint exists, we deny the proposed movement and force agent to wait one timestep                
                    continue                                                                                #by returning the agent as a child to the OPEN list with the same location but with timestep + 1

                #check negative constraints for vertices and edges
                if path_constrained(child_loc, curr['timestep'] +1, ver_path_constraint_table):
                    continue
                if edge_constrained(curr['loc'], child_loc, curr['timestep'] + 1,
                                    edge_path_constraint_table):
                    continue
                    
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}                                                   #added an addition to update timestep in child
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
