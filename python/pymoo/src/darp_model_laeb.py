from pyomo.environ import *
import itertools

class DARPModelLAEB:
    """
    Static Dial-A-Ride Problem (DARP) - Location-Augmented-Event-Based (LAEB).
    Basado en Gaul et al. (2025).
    """

    def __init__(self, data):
        """
        Inicia el modelo LAEB.
        :param data: Diccionario con localizaciones, peticiones, y la estructura 
                     del grafo de eventos (Event-Based Graph: V y A).
        """
        self.data = data
        self.model = ConcreteModel()
        self.validate_data()
        self._build_model()

    def validate_data(self):
        """
        Valida que los datos necesarios para la estructura de eventos estén presentes.
        """
        required_keys = [
            'V', 'A', 'Requests', 'P', 'D', 'J', 'costs', 'travel_times',
            'service_times', 'tw_start', 'tw_end', 'max_ride_time', 'vehicles'
        ]
        for key in required_keys:
            if key not in self.data:
                raise ValueError(f"Falta la clave de datos requerida: {key}")

    def _build_model(self):
        m = self.model

        # --- Conjuntos ---
        m.V = Set(initialize=self.data['V']) # Nodos de eventos (tuplas de estado) [cite: 202]
        m.A = Set(initialize=self.data['A']) # Arcos entre eventos [cite: 214]
        m.R = Set(initialize=self.data['Requests']) # Peticiones {1...n} [cite: 101]
        m.J = Set(initialize=self.data['J']) # Localizaciones {0, P, D} [cite: 103]
        
        # Subconjuntos de eventos para mapeo
        # V_i+ y V_i- son eventos donde la ubicación actual es pickup o delivery de i [cite: 192, 197]
        m.V_pickup = {i: [v for v in m.V if v[0] == f"{i}+"] for i in m.R}
        m.V_delivery = {i: [v for v in m.V if v[0] == f"{i}-"] for i in m.R}

        # --- Parámetros ---
        m.c = Param(m.A, initialize=self.data['costs']) # Costos de los arcos [cite: 230]
        m.t = Param(m.A, initialize=self.data['travel_times']) # Tiempos de viaje [cite: 230]
        m.s = Param(m.J, initialize=self.data['service_times']) # Tiempos de servicio [cite: 107]
        m.e = Param(m.J, initialize=self.data['tw_start']) # Inicio ventana horaria [cite: 106]
        m.l = Param(m.J, initialize=self.data['tw_end']) # Fin ventana horaria [cite: 106]
        m.L_max = Param(m.R, initialize=self.data['max_ride_time']) # Tiempo máximo de viaje [cite: 103]
        m.K = Param(initialize=self.data['vehicles']) # Número de vehículos [cite: 102]

        # --- Variables ---
        m.x = Var(m.A, domain=Binary) # Variable de decisión del arco (v,w) [cite: 231, 250]
        m.B_bar = Var(m.J, domain=NonNegativeReals) # Inicio de servicio en ubicación j [cite: 294, 302]

        # --- Objetivo (3a) ---
        # Minimizar los costos totales de ruta[cite: 296].
        m.obj = Objective(expr=sum(m.c[a] * m.x[a] for a in m.A), sense=minimize)

        # --- Restricciones ---

        # (2b) Conservación de flujo en el grafo de eventos [cite: 239, 273]
        @m.Constraint(m.V)
        def flow_conservation(m_in, v):
            # No aplicar al depósito si se maneja como inicio/fin específico
            if v == ('0',): return Constraint.Skip 
            in_flow = sum(m_in.x[u, v_inner] for (u, v_inner) in m_in.A if v_inner == v)
            out_flow = sum(m_in.x[v_inner, w] for (v_inner, w) in m_in.A if v_inner == v)
            return in_flow - out_flow == 0

        # (2c) Visitar cada ubicación de recogida exactamente una vez [cite: 242, 274]
        @m.Constraint(m.R)
        def visit_pickups(m_in, i):
            arcs_to_pickup = [a for a in m_in.A if a[1] in m.V_pickup[i]]
            return sum(m_in.x[a] for a in arcs_to_pickup) == 1

        # (2d) Límite de flota desde el depósito [cite: 243, 276]
        @m.Constraint()
        def fleet_limit(m_in):
            depot_event = ('0',)
            out_arcs = [a for a in m_in.A if a[0] == depot_event]
            return sum(m_in.x[a] for a in out_arcs) <= m_in.K

        # (3b) Consistencia de tiempo (Big-M mejorado para LAEB) [cite: 299, 305]
        @m.Constraint(m.J, m.J)
        def time_consistency(m_in, i, j):
            # Suma de flujos entre ubicaciones físicas i y j en el grafo de eventos
            relevant_arcs = [a for a in m_in.A if a[0][0] == i and a[1][0] == j]
            if not relevant_arcs: return Constraint.Skip
            
            # Big-M ajustado [cite: 309]
            M_ij = m_in.l[i] + m_in.s[i] + self.data['travel_matrix'][i][j] - m_in.e[j]
            return m_in.B_bar[j] >= m_in.B_bar[i] + m_in.s[i] + \
                   self.data['travel_matrix'][i][j] - M_ij * (1 - sum(m_in.x[a] for a in relevant_arcs))

        # (1h) Ventanas horarias en ubicaciones [cite: 152, 303]
        @m.Constraint(m.J)
        def time_windows(m_in, j):
            return (m_in.e[j], m_in.B_bar[j], m_in.l[j])

        # (1j) Tiempo máximo de viaje para pasajeros [cite: 154, 165]
        @m.Constraint(m.R)
        def max_ride_time(m_in, i):
            p = f"{i}+"
            d = f"{i}-"
            return m_in.B_bar[d] - (m_in.B_bar[p] + m_in.s[p]) <= m_in.L_max[i]

    def solve(self, solver_name='cplex', executable_path=None):
        """
        Resuelve el modelo MILP.
        """
        solver = SolverFactory(solver_name, executable=executable_path)
        results = solver.solve(self.model, tee=True)
        return results

    def get_route_summary(self):
        """
        Extrae las rutas del grafo de eventos traduciéndolas a ubicaciones físicas.
        """
        if not hasattr(self.model, 'x'):
            return "Modelo no resuelto."

        active_arcs = [a for a in self.model.A if value(self.model.x[a]) > 0.5]
        
        # Reconstrucción de rutas por vehículo
        routes = []
        starts = [a for a in active_arcs if a[0] == ('0',)]
        
        for start_arc in starts:
            route = [start_arc[0][0], start_arc[1][0]]
            times = [value(self.model.B_bar[route[0]]), value(self.model.B_bar[route[1]])]
            curr = start_arc[1]
            
            while True:
                next_arc = next((a for a in active_arcs if a[0] == curr), None)
                if not next_arc or next_arc[1][0] == '0':
                    if next_arc:
                        route.append('0')
                        times.append(value(self.model.B_bar['0']))
                    break
                curr = next_arc[1]
                route.append(curr[0])
                times.append(value(self.model.B_bar[curr[0]]))
            
            routes.append({'path': route, 'times': times})
        return routes

    def print_route_summary(self):
        """
        Imprime el resumen de las rutas LAEB.
        """
        summary = self.get_route_summary()
        if isinstance(summary, str):
            print(summary)
            return
            
        print("--- Resumen de Rutas (Formulación LAEB) ---")
        for idx, r in enumerate(summary):
            print(f"Vehículo {idx + 1}:")
            path_str = " -> ".join([f"{loc} (T:{t:.2f})" for loc, t in zip(r['path'], r['times'])])
            print(f"  Ruta: {path_str}")