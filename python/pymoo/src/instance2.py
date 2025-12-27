import math
from pyomo.environ import value
from pyomo.opt import SolverFactory, TerminationCondition
# Assuming the class is saved in a file named darp_model_lb.py
from darp_model import DARPModelLB

def get_demanding_instance():
    """
    Genera una instancia DARP con 7 solicitudes (16 nodos totales).
    Incluye coordenadas para distancias realistas y ventanas de tiempo escalonadas.
    """
    requests = list(range(1, 8))  # R = {1, 2, 3, 4, 5, 6, 7}
    pickups = [f"{r}+" for r in requests]
    deliveries = [f"{r}-" for r in requests]
    nodes = pickups + deliveries
    all_nodes = ['0_start'] + nodes + ['0_end']

    # Coordenadas (x, y) para simular un área de 100x100
    # Esto genera tiempos de viaje heterogéneos (no solo 1s y 5s)
    coords = {
        '0_start': (50, 50), '0_end': (50, 50),
        '1+': (10, 15), '1-': (25, 30),
        '2+': (85, 10), '2-': (70, 20),
        '3+': (15, 85), '3-': (40, 75),
        '4+': (90, 90), '4-': (75, 70),
        '5+': (10, 50), '5-': (30, 55),
        '6+': (60, 80), '6-': (50, 95),
        '7+': (45, 10), '7-': (55, 35)
    }

    def get_dist(n1, n2):
        return round(math.sqrt((coords[n1][0]-coords[n2][0])**2 + 
                               (coords[n1][1]-coords[n2][1])**2), 1)

    travel_times = {(i, j): get_dist(i, j) for i in all_nodes for j in all_nodes}

    # Ventanas de tiempo (e_j, l_j) escalonadas para forzar la planificación
    tw_start = {'0_start': 0, '0_end': 0}
    tw_end = {'0_start': 240, '0_end': 240}
    
    # Horarios de recogida distribuidos
    pickup_times = {
        '1+': (0, 30), '2+': (10, 40), '3+': (20, 50), 
        '4+': (30, 60), '5+': (40, 70), '6+': (50, 80), '7+': (60, 90)
    }

    for p, (start, end) in pickup_times.items():
        tw_start[p], tw_end[p] = start, end
        # Las entregas tienen ventanas más amplias que dependen de la recogida
        r_idx = p[0]
        tw_start[f"{r_idx}-"] = start + 5
        tw_end[f"{r_idx}-"] = end + 60

    data = {
        'Requests': requests,
        'P': pickups,
        'D': deliveries,
        'costs': travel_times,
        'travel_times': travel_times,
        'service_times': {n: 2 for n in nodes} | {'0_start': 0, '0_end': 0}, # s_j = 2 min
        'load_change': {
            **{'0_start': 0, '0_end': 0}, 
            **{p: 1 for p in pickups}, 
            **{d: -1 for d in deliveries}
        },
        'tw_start': tw_start,
        'tw_end': tw_end,
        'max_ride_time': {r: 40 for r in requests}, # L_i = 40 min
        'capacity': 3,
        'vehicles': 2  # Usar 2 vehículos hace el problema más interesante que solo 1
    }
    return data

if __name__ == "__main__":
    # Load instance data
    instance_data = get_demanding_instance()
    
    # Initialize the model using the LB class structure
    darp_instance = DARPModelLB(instance_data)
    
    try:
        darp_instance.solve(solver_name='gurobi')
        darp_instance.print_route_summary()
    except Exception as e:
        print(f"An error occurred while solving the model: {e}")
