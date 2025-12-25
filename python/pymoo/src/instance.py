"""
Main execution script for the Dial-A-Ride Problem (DARP).
This script initializes instance data based on literature benchmarks 
and utilizes the DARPModelLB class for optimization.
"""

from pyomo.environ import value
from pyomo.opt import SolverFactory, TerminationCondition
# Assuming the class is saved in a file named darp_model_lb.py
from darp_model import DARPModelLB

def get_example_instance():
    """
    Generates a sample DARP instance with 3 requests.
    Parameters are based on Example 1 from the source literature.
    """
    # Define request, pickup, and delivery sets
    requests = [1, 2, 3] # R = {1, 2, 3}
    pickups = [f"{r}+" for r in requests] # P = {1+, 2+, 3+}
    deliveries = [f"{r}-" for r in requests] # D = {1-, 2-, 3-}
    nodes = pickups + deliveries
    all_nodes = ['0_start'] + nodes + ['0_end']

    # Generate travel time matrix based on Table 1
    # Costs are symmetric; travel to/from depot is 5, between customers is 1
    travel_times = {}
    for i in all_nodes:
        for j in all_nodes:
            if i == j: 
                travel_times[i, j] = 0
            elif '0' in i or '0' in j: 
                travel_times[i, j] = 5 # travel time to depot
            else: 
                travel_times[i, j] = 1 # travel time between nodes

    # Model parameters assembly
    data = {
        'Requests': requests,
        'P': pickups,
        'D': deliveries,
        'costs': travel_times,         # Objective (1a) coefficients
        'travel_times': travel_times,  # t_ij for time continuity
        'service_times': {n: 0 for n in all_nodes}, # s_j = 0
        'load_change': {
            **{'0_start': 0, '0_end': 0}, 
            **{f"{r}+": 1 for r in requests},  # q_i = 1 for pickups
            **{f"{r}-": -1 for r in requests}  # q_i = -1 for deliveries
        },
        'tw_start': {
            **{'0_start': 0, '0_end': 0}, 
            **{n: 5 for n in nodes}   # e_j = 5
        },
        'tw_end': {
            **{'0_start': 100, '0_end': 100}, 
            **{n: 60 for n in nodes}  # l_j = 60
        },
        'max_ride_time': {r: 100 for r in requests}, # L_i = 100
        'capacity': 3,  # Q = 3
        'vehicles': 1   # K = 1
    }
    return data

if __name__ == "__main__":
    # Load instance data
    instance_data = get_example_instance()
    
    # Initialize the model using the LB class structure
    darp_instance = DARPModelLB(instance_data)
    
    try:
        darp_instance.solve(solver_name='gurobi')
        darp_instance.print_route_summary()
    except Exception as e:
        print(f"An error occurred while solving the model: {e}")

