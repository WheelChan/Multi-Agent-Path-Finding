# Multi-Agent-Path-Finding
 Goal of project is to write programs that will allow multiple agents to find optimal paths from their start location to their goal location such that all agents are able to move simultaneously. Various algorithms will be explored such as Prioritized Planning, Conflict-Based Search (CBS), A* Search. Additionally, a paper that describes Safe Interval Path Planning (SIPP) provided the inspiration for writing of SIPP versions of Prioritized Planning and CBS.

NOTE: The original paper detailing and describing SIPP is included here in the file Original_Paper.pdf

# HOW TO USE CODE:
1. independent.py represents the code for path planning for a single agent. 
2. cbs.py and prioritized.py represent CBS Search and Prioritized Planning respectively.
3. Instances for path planning are provided in the instances folder
         - the line used to run code is as follows: python run_experiments.py --instance instances/test1.txt --solver Independent
         - the line above would use the independent solver in independent.py to solve the instance desribed in test1.txt
         - changing 'Independent' to 'Prioritized' or 'CBS' will change the solver, additional solvers are described in run_experiements.py
4. SIPP versions of CBS, Prioritized Planning, and Independent solvers are represented by the files labeled sipp____ .py


# POTENTIAL ISSUES:
1. CBS works for most cases but not all, the same for SIPP CBS but CBS supposed to be an optimal and complete algorithm so issue may be in how we choose to expand the next search iteration. 
