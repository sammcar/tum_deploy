import numpy as np
import matplotlib.pyplot as plt

def matrix_bezier(P0,P1,P2,P3,t):

    """
    Forma en matriz para Bézier Cúbica. // t: va de 0.0 a 1.0 // P0, P1, P2, P3: Puntos de control de la curva np.array([x,y,z])
    """

    t_potencias = np.array([1,t,t**2,t**3])
    puntos_control = np.array([P0,P1,P2,P3])

    matriz_caracteristica = np.array([
        [ 1,  0,  0,  0],
        [-3,  3,  0,  0],
        [ 3, -6,  3,  0],
        [-1,  3, -3,  1]
    ])

    return t_potencias @ matriz_caracteristica @ puntos_control

def calcular_puntos_control_paso(inicio, fin, altura_deseada):
    """
    Calcula P0, P1, P2, P3 para un paso con altura real exacta. // inicio, fin: np.array([x, y, z] // altura_deseada: float (ej. 0.05 para 5cm)
    """
    P0 = np.array(inicio)
    P3 = np.array(fin)
    z_suelo = inicio[2]
    h_ctrl = z_suelo + (4.0 / 3.0) * altura_deseada
    P1 = np.array([inicio[0], inicio[1], h_ctrl])
    P2 = np.array([fin[0], fin[1], h_ctrl])
    
    return P0, P1, P2, P3


def lerp(P0,P1,t): 
    """
    Interpolación lineal // t: va de 0.0 a 1.0 // P0: Punto Inicio // P1: Punto Final
    """
    return ((1-t)*P0)+(t*P1) 


def generar_paso(inicio, final, altura, t):
    """
    Generador de Pasos Simple (50 DuttyCycle) // t: va de 0.0 a 2.0 // inicio: Punto inicio (Pie en suelo atrás) // final: Punto final (Pie en suelo adelante) // altura: Altura máxima del paso
    """

    P0, P1, P2, P3 = calcular_puntos_control_paso (inicio, final, altura)
    if t <= 1:
        return matrix_bezier (P0, P1, P2, P3, t)
    else:
        return lerp(P3,P0,t-1)

def get_curve_point(inicio, final, altura, t):
    """
    Función pura: Dame inicio, fin, altura y % del recorrido (0 a 1)
    y te doy la coordenada 3D.
    """
    P0, P1, P2, P3 = calcular_puntos_control_paso(inicio, final, altura)
    return matrix_bezier(P0, P1, P2, P3, t)

# # VISUALIZER

# # Configuración de la trayectoria
# pos_inicio = np.array([-0.06, 0.0, 0.0])  # Pie atrás (10cm atrás)
# pos_final = np.array([0.06, 0.0, 0.0])   # Pie adelante (10cm adelante)
# h_paso = 0.03                      # Altura 5cm

# # Generar puntos para visualización
# t_values = np.linspace(0, 2, 100)
# puntos = np.array([generar_paso(pos_inicio, pos_final, h_paso, t) for t in t_values])

# # Separar fases para colorear
# swing_points = puntos[t_values <= 1.0]
# stance_points = puntos[t_values > 1.0]

# # Graficar
# plt.figure(figsize=(10, 6))
# plt.plot(swing_points[:, 0], swing_points[:, 2], 'b-', lw=3, label='Fase de Vuelo (Bézier)')
# plt.plot(stance_points[:, 0], stance_points[:, 2], 'r--', lw=3, label='Fase de Apoyo (LERP)')

# # Dibujar puntos de control
# P0, P1, P2, P3 = calcular_puntos_control_paso(pos_inicio, pos_final, h_paso)
# plt.scatter([P0[0], P1[0], P2[0], P3[0]], [P0[2], P1[2], P2[2], P3[2]], color='green', zorder=5, label='Puntos de Control')
# plt.text(P0[0], P0[2]-0.005, 'P0', ha='center')
# plt.text(P1[0], P1[2]+0.005, 'P1', ha='center')
# plt.text(P2[0], P2[2]+0.005, 'P2', ha='center')
# plt.text(P3[0], P3[2]-0.005, 'P3', ha='center')

# plt.title("Ciclo Completo de una Pata (Vista Lateral X-Z)")
# plt.xlabel("Posición X (Avance)")
# plt.ylabel("Posición Z (Altura)")
# plt.axhline(0, color='black', lw=1, alpha=0.5)
# plt.grid(True, linestyle=':', alpha=0.7)
# plt.legend()
# plt.axis('equal')
# plt.show()
