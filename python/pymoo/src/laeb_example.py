from darp_model_laeb import DARPModelLAEB

# --- DATOS DE EJEMPLO PARA LA FORMULACIÓN LAEB ---
# Request 1: de '1+' (recogida) a '1-' (entrega)
test_data = {
    'Requests': [1],
    'P': ['1+'],
    'D': ['1-'],
    'J': ['0', '1+', '1-'], # Localizaciones físicas (depósito, pickup, delivery)
    'vehicles': 1,
    'capacity': 3,
    
    # Grafo de Eventos (V): Tuplas (ubicación_actual, pasajeros_a_bordo...)
    # Usamos 0 para representar que el vehículo está vacío en esa posición de la tupla.
    'V': [
        ('0',),       # Evento inicial: Depósito vacío
        ('1+', 1),    # Evento: En pickup 1, pasajero 1 a bordo
        ('1-', 0),    # Evento: En delivery 1, ya no hay pasajeros (vacío)
    ],
    
    # Arcos de Eventos (A): Transiciones factibles entre estados [cite: 214]
    'A': [
        (('0',), ('1+', 1)),  # Salida del depósito hacia recogida 1
        (('1+', 1), ('1-', 0)), # De recogida 1 a entrega 1
        (('1-', 0), ('0',))    # Regreso al depósito tras entregar
    ],
    
    # Tiempos de viaje entre localizaciones físicas (para la restricción 3b) [cite: 299]
    'travel_matrix': {
        '0':  {'0': 0, '1+': 5, '1-': 10},
        '1+': {'0': 5, '0': 0, '1-': 8},
        '1-': {'0': 5, '1+': 8, '0': 0}
    },
    
    # Los costos y tiempos de los ARCOS (A) se derivan de la matriz física [cite: 230]
    'costs': {
        (('0',), ('1+', 1)): 5,
        (('1+', 1), ('1-', 0)): 8,
        (('1-', 0), ('0',)): 5
    },
    'travel_times': {
        (('0',), ('1+', 1)): 5,
        (('1+', 1), ('1-', 0)): 8,
        (('1-', 0), ('0',)): 5
    },
    
    'service_times': {'0': 0, '1+': 2, '1-': 2},
    'tw_start': {'0': 0, '1+': 0, '1-': 0},
    'tw_end': {'0': 100, '1+': 50, '1-': 80},
    'max_ride_time': {1: 30} # Pasajero 1 puede estar máximo 30 min [cite: 103, 154]
}

# --- EJECUCIÓN DEL MODELO ---

# 1. Instanciar (Asegúrate de tener la clase DARPModelLAEB definida antes)
modelo_laeb = DARPModelLAEB(test_data)

# 2. Resolver 
# Nota: Requiere tener instalado un solver como 'glpk', 'cbc' o 'cplex'
results = modelo_laeb.solve(solver_name='glpk') 

# 3. Mostrar Resultados
print("\n--- RESULTADOS DEL TEST ---")
modelo_laeb.print_route_summary()