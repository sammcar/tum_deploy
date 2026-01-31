import numpy as np

def solve_IK(x, y, z, L1, L2, L3, es_pata_derecha, es_tipo_rodilla = False):
    """
    x, y, z: Coordenadas del pie
    es_pata_derecha: True si es derecha (cambia signo theta1)
    es_tipo_rodilla: True para rodilla hacia atras (>), False para codo (<)
    """
    y_local = -y if es_pata_derecha else y

    # Seguridad
    dist_yz = np.sqrt(y_local**2 + z**2)
    min_dist_yz = L1 + 0.001
    if dist_yz < min_dist_yz:
        scale_yz = min_dist_yz / (dist_yz + 1e-9)
        y_local *= scale_yz
        z *= scale_yz
        dist_yz = min_dist_yz

    R_check = np.sqrt(np.maximum(dist_yz**2 - L1**2, 0))
    dist_total = np.sqrt(x**2 + R_check**2)
    max_reach = (L2 + L3) * 0.999
    
    if dist_total > max_reach:
        scale = max_reach / dist_total
        x *= scale
        R_new = R_check * scale
        D_new = np.sqrt(R_new**2 + L1**2)
        scale_yz_final = D_new / dist_yz
        y_local *= scale_yz_final
        z *= scale_yz_final

    # Cadera
    D = np.maximum(np.sqrt(y_local**2 + z**2),L1)
    theta1 = (np.arctan2(z, y_local) + np.arccos(L1 / D)) if D > 1e-6 else 0.0

    # Rodilla

    R = -np.sqrt(np.maximum(D**2 - L1**2,0)) # Negativo hacia abajo
    H = np.minimum(np.sqrt(x**2 + R**2), L2 + L3 - 1e-6)

    phi1 = np.arccos(np.clip((L2**2 + L3**2 - H**2) / (2 * L2 * L3), -1.0, 1.0)) # Siempre positivo entre 0 y 180
    
    if es_tipo_rodilla:
        theta3 = phi1 - np.pi 
        signo_phi3 = 1
    else:
        theta3 = np.pi - phi1 
        signo_phi3 = -1

    # Muslo
    phi2 = np.arctan2(R, x)
    phi3 = np.arcsin(np.clip((L3 * np.sin(phi1)) / H, -1.0, 1.0))
    theta2 = phi2 + (signo_phi3 * phi3)

    if es_pata_derecha: theta1 = -theta1

    return wrap_to_pi(theta1), wrap_to_pi(theta2), wrap_to_pi(theta3)

def dh_transform(theta, d, a, alpha):
    """ Matriz DH Estándar """
    c, s = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -s * ca,  s * sa, a * c],
        [s,  c * ca, -c * sa, a * s],
        [0,      sa,      ca,     d],
        [0,       0,       0,     1]
    ])

def solve_FK(theta1, theta2, theta3, L1, L2, L3, es_pata_derecha, es_tipo_rodilla=False):
    """
    FK usando tabla MRPT.
    """
    l1_eff = -L1 if es_pata_derecha else L1
    
    T1 = dh_transform(np.pi/2, 0, 0, np.pi/2)
    T2 = dh_transform(theta1, 0, l1_eff, np.pi/2)
    T3 = dh_transform(np.pi/2, 0, 0, -np.pi/2)
    T4 = dh_transform(theta2, 0, L2, 0)
    T5 = dh_transform(theta3, 0, L3, 0)
    
    P0 = np.array([0,0,0,1])

    p0 = P0[:3]
    p1 = (T1 @ T2 @ P0)[:3]                 # Cadera
    p2 = (T1 @ T2 @ T3 @ T4 @ P0)[:3]      # Rodilla
    p3 = (T1 @ T2 @ T3 @ T4 @ T5 @ P0)[:3] # Pie
    
    return [p0, p1, p2, p3]

def origen_a_cadera_local(coord_pie, cuerpo_rpy, hip_offset, cuerpo_pos=np.array([0,0,0]), centro_rotacion=np.array([0,0,0])):
    """
    Traduce la posición del pie del origen al sistema local de la cadera.
    """
    R_cuerpo = get_rotation_matrix(cuerpo_rpy[0], cuerpo_rpy[1], cuerpo_rpy[2])
    compensacion_pivote = np.array(centro_rotacion) - (R_cuerpo @ np.array(centro_rotacion))
    hip_origen = np.array(cuerpo_pos) + compensacion_pivote + (R_cuerpo @ np.array(hip_offset))
    vector_origen = np.array(coord_pie) - hip_origen
    vector_local = R_cuerpo.T @ vector_origen
    
    return vector_local

def wrap_to_pi(angle):
    """
    Fuerza a que el ángulo esté siempre entre -PI y PI (-180 a 180)
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def get_rotation_matrix(r, p, y):
    """ Genera la matriz de rotación 3x3 (Global Body Rotation) """
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx