from pyomo.environ import *
import itertools

class DARPModelLB:
    """
    Static Dial-A-Ride Problem (DARP) - Location-Based (LB) MILP Formulation.
    Based on the model by Ropke et al. (2007).
    """

    def __init__(self, data):
        """
        Initializes the DARP model with input data.
        :param data: Dictionary containing sets, travel times, costs, and constraints.
        """
        self.data = data
        self.model = ConcreteModel()
        self.validate_data()
        self._build_model()
    
    def validate_data(self):
        """
        Validates the input data for consistency and completeness.
        """
        required_keys = [
            'Requests', 'P', 'D', 'costs', 'travel_times', 
            'service_times', 'load_change', 'tw_start', 
            'tw_end', 'max_ride_time', 'capacity', 'vehicles'
        ]
        # Check for missing keys
        for key in required_keys:
            if key not in self.data:
                raise ValueError(f"Missing required data key: {key}")

    def _generate_s_sets(self):
        """
        Generates subsets S for pairing and precedence constraints (1e).
        WARNING: This set grows exponentially with the number of customers (n).
        """
        all_nodes = list(self.data['P']) + list(self.data['D'])
        start_node = '0_start'
        valid_S_sets = []

        # Iterate through all possible subset sizes for customer nodes
        for r in range(1, len(all_nodes) + 1):
            for subset in itertools.combinations(all_nodes, r):
                S = set(subset)
                S.add(start_node)
                
                # Condition (1e): Subset must include delivery node i- but not pickup node i+.
                has_delivery_without_pickup = False
                for i in self.data['Requests']:
                    pickup = f"{i}+"
                    delivery = f"{i}-"
                    if (delivery in S) and (pickup not in S):
                        has_delivery_without_pickup = True
                        break
                
                if has_delivery_without_pickup:
                    valid_S_sets.append(tuple(S))
        return valid_S_sets

    def _build_model(self):
        m = self.model

        # --- Sets ---
        m.Requests = Set(initialize=self.data['Requests']) # 
        m.P = Set(initialize=self.data['P']) # Pickup locations
        m.D = Set(initialize=self.data['D']) # Delivery locations
        m.J_minus_depot = m.P | m.D
        m.J = Set(initialize=['0_start'] + list(m.J_minus_depot) + ['0_end']) # All nodes

        # --- Valid Arcs ---
        # Binary variable x[i,j] is 1 if a vehicle drives directly from i to j.
        m.A = Set(initialize=[(i, j) for i in m.J for j in m.J 
                             if i != j and i != '0_end' and j != '0_start'])

        # --- Parameters ---
        m.c = Param(m.J, m.J, initialize=self.data['costs']) # Routing costs 
        m.t = Param(m.J, m.J, initialize=self.data['travel_times']) # Travel times 
        m.s = Param(m.J, initialize=self.data['service_times']) # Service times 
        m.q = Param(m.J, initialize=self.data['load_change']) # Load change at location 
        m.e = Param(m.J, initialize=self.data['tw_start']) # Time window start 
        m.l = Param(m.J, initialize=self.data['tw_end']) # Time window end 
        m.L = Param(m.Requests, initialize=self.data['max_ride_time']) # Max ride time 
        m.Q_max = Param(initialize=self.data['capacity']) # Vehicle capacity 
        m.K = Param(initialize=self.data['vehicles']) # Number of vehicles 

        # --- Variables ---
        m.x = Var(m.A, domain=Binary) # Sequence variables 
        m.B = Var(m.J, domain=NonNegativeReals) # Beginning of service 
        m.Q = Var(m.J, domain=NonNegativeReals) # Vehicle load after leaving location 

        # --- Objective (1a) ---
        # Minimize total routing costs.
        m.obj = Objective(expr=sum(m.c[i, j] * m.x[i, j] for (i, j) in m.A), sense=minimize)

        # --- Constraints ---

        # (1b) & (1c) - Node Visitation
        # Each customer location must be visited and left exactly once.
        @m.Constraint(m.J_minus_depot)
        def visit_once(m_in, j):
            return sum(m_in.x[i, j] for i in m_in.J if (i, j) in m_in.A) == 1

        @m.Constraint(m.J_minus_depot)
        def leave_once(m_in, i):
            return sum(m_in.x[i, j] for j in m_in.J if (i, j) in m_in.A) == 1

        # (1d) - Fleet Limit
        # At most K vehicles leave the starting depot.
        @m.Constraint()
        def fleet_limit(m_in):
            return sum(m_in.x['0_start', j] for j in m_in.P if ('0_start', j) in m_in.A) <= m_in.K

        # (1e) - Pairing and Precedence
        # Ensures pickup and delivery are in the same tour in the correct order.
        s_sets = self._generate_s_sets()
        m.S_Indices = Set(initialize=range(len(s_sets)))
        m.S_Sets = Param(m.S_Indices, initialize={idx: val for idx, val in enumerate(s_sets)})

        @m.Constraint(m.S_Indices)
        def precedence_pairing(m_in, idx):
            S = m_in.S_Sets[idx]
            arcs_in_S = [(i, j) for i in S for j in S if (i, j) in m_in.A]
            return sum(m_in.x[i, j] for (i, j) in arcs_in_S) <= len(S) - 2

        # (1f) - Time Continuity
        # Ensures time consistency between successive locations (Big-M).
        @m.Constraint(m.A)
        def time_consistency(m_in, i, j):
            M_ij = m_in.l[i] + m_in.s[i] + m_in.t[i, j] - m_in.e[j]
            return m_in.B[i] + m_in.s[i] + m_in.t[i, j] - M_ij * (1 - m_in.x[i, j]) <= m_in.B[j]

        # (1g) - Load Continuity
        # Models vehicle load progression and capacity limits.
        @m.Constraint(m.A)
        def load_consistency(m_in, i, j):
            return m_in.Q[i] + m_in.q[j] - m_in.Q_max * (1 - m_in.x[i, j]) <= m_in.Q[j]

        # (1h) - Time Windows
        # Start of service must be within specified windows.
        @m.Constraint(m.J)
        def service_windows(m_in, j):
            return (m_in.e[j], m_in.B[j], m_in.l[j])

        # (1i) - Capacity Constraints
        # Load cannot exceed vehicle capacity Q.
        @m.Constraint(m.J)
        def capacity_bound(m_in, j):
            lower = max(0, m_in.q[j])
            upper = min(m_in.Q_max, m_in.Q_max + m_in.q[j])
            return (lower, m_in.Q[j], upper)

        # (1j) - Maximum Ride Time
        # Restricts the time a customer spends in the vehicle to L_i.
        @m.Constraint(m.Requests)
        def max_ride_duration(m_in, r):
            p = f"{r}+"
            d = f"{r}-"
            return m_in.B[d] - (m_in.B[p] + m_in.s[p]) <= m_in.L[r]

    def solve(self, solver_name='gurobi', executable_path=None):
        """
        Solves the MILP model using the specified solver.
        """
        solver = SolverFactory(solver_name, executable=executable_path)
        results = solver.solve(self.model, tee=True)
        return results
    
    def get_route_summary(self):
        """ 
        Extracts results into a clean dictionary with route, time, and load info.
        """
        if not hasattr(self.model, 'x'):
            return "Model not solved yet."
            
        routes = []
        for (i, j) in self.model.x:
            if value(self.model.x[i, j]) > 0.5:
                routes.append({
                    'from': i,
                    'to': j,
                    'depart_time': value(self.model.B[i]),
                    'arrive_time': value(self.model.B[j]),
                    'load_after': value(self.model.Q[j])
                })
        # Group by vehicle route (simple heuristic: start at depot)
        summary = {}
        for route in routes:
            if route['from'] == '0_start':
                current = route['to']
                path = [route['from'], current]
                times = [route['depart_time'], route['arrive_time']]
                loads = [0, route['load_after']]
                while current != '0_end':
                    next_leg = next((r for r in routes if r['from'] == current), None)
                    if not next_leg:
                        break
                    current = next_leg['to']
                    path.append(current)
                    times.append(next_leg['arrive_time'])
                    loads.append(next_leg['load_after'])
                summary[path[1]] = {
                    'route': path,
                    'times': times,
                    'loads': loads
                }
        return summary

    def print_route_summary(self):
        """
        Prints the route summary in a readable format.
        """
        summary = self.get_route_summary()
        if isinstance(summary, str):
            print(summary)
            return
        for vehicle, info in summary.items():
            print(f"Vehicle starting at {info['route'][0]}:")
            for idx in range(len(info['route']) - 1):
                print(f"  From {info['route'][idx]} to {info['route'][idx + 1]} | "
                      f"Depart: {info['times'][idx]:.2f}, Arrive: {info['times'][idx + 1]:.2f}, "
                      f"Load after: {info['loads'][idx + 1]:.2f}")
            print("")

   