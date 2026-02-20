import pandas as pd
import matplotlib.pyplot as plt
import os

# Ruta exacta donde guardaste el CSV
ruta_archivo = os.path.expanduser("/home/tum/tum/sam_cpp/build/torques_log.csv")

print(f"Buscando datos en: {ruta_archivo}")

# Leer el archivo CSV
try:
    df = pd.read_csv(ruta_archivo)
    print(f"¡Datos cargados con éxito! {len(df)} muestras encontradas.")
except FileNotFoundError:
    print(f"❌ Error: No se encontró el archivo. Asegúrate de haber ejecutado el robot primero.")
    exit()

# Nombres de las patas para los títulos
nombres_patas = {0: "Pata 0 (Front-Left)", 1: "Pata 1 (Front-Right)", 
                 2: "Pata 2 (Back-Left)", 3: "Pata 3 (Back-Right)"}

# Crear una figura grande con 4 subgráficas (2 filas x 2 columnas)
fig, axes = plt.subplots(2, 2, figsize=(16, 10), sharex=True)
axes = axes.flatten() # Para iterar fácilmente sobre los 4 ejes

for leg_id in range(4):
    # Filtrar solo los datos de la pata actual
    datos_pata = df[df['Leg_ID'] == leg_id].reset_index(drop=True)
    
    ax = axes[leg_id]
    
    if datos_pata.empty:
        ax.set_title(f"{nombres_patas[leg_id]} - Sin datos")
        continue

    # Graficar los torques en el eje Y principal (Izquierdo)
    l1 = ax.plot(datos_pata.index, datos_pata['Torque_Femur'], label='Torque Fémur', color='blue', alpha=0.8, linewidth=1.5)
    l2 = ax.plot(datos_pata.index, datos_pata['Torque_Tibia'], label='Torque Tibia', color='red', alpha=0.8, linewidth=1.5)
    ax.set_ylabel('Torque (Nm)')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_title(nombres_patas[leg_id], fontsize=14, fontweight='bold')

    # Crear un eje Y secundario (Derecho) para la fase de la pata
    ax2 = ax.twinx()
    l3 = ax2.plot(datos_pata.index, datos_pata['Phase'], label='Fase (0 a 1)', color='green', linestyle=':', alpha=0.7)
    ax2.set_ylabel('Fase')
    ax2.set_ylim(0, 1.1)

    # También podemos colorear el fondo cuando 'Is_Stance' es True (suelo)
    ax.fill_between(datos_pata.index, ax.get_ylim()[0], ax.get_ylim()[1], 
                    where=datos_pata['Is_Stance'], color='gray', alpha=0.15, label='Fase Stance (Teórica)')

    # Unir todas las leyendas en un solo cuadro
    lineas = l1 + l2 + l3
    etiquetas = [l.get_label() for l in lineas]
    # Agregamos manualmente la etiqueta del fondo gris
    import matplotlib.patches as mpatches
    patch = mpatches.Patch(color='gray', alpha=0.15, label='Fase Stance (Suelo)')
    ax.legend(handles=lineas + [patch], loc='upper left', fontsize=9)

# Ajustes finales de la ventana
plt.suptitle("Análisis de Torques por Pata durante la Marcha", fontsize=18)
fig.text(0.5, 0.04, 'Muestras de Tiempo (Ciclos de Control)', ha='center', fontsize=12)
plt.tight_layout(rect=[0, 0.05, 1, 0.95]) # Deja espacio para el título principal

print("\n" + "="*55)
print(" 📊 ANÁLISIS ESTADÍSTICO DE TORQUES PARA UMBRALES")
print("="*55)

# Usamos el valor absoluto porque el motor puede aplicar torque negativo o positivo
df['Torque_Femur_Abs'] = df['Torque_Femur'].abs()
df['Torque_Tibia_Abs'] = df['Torque_Tibia'].abs()

for leg_id in range(4):
    datos_pata = df[df['Leg_ID'] == leg_id]
    if datos_pata.empty:
        continue

    # Filtrar por fase: Swing (Is_Stance == False) y Stance (Is_Stance == True)
    # Además, para el swing recortamos un poco para no tomar el instante exacto del despegue/aterrizaje
    swing_puro = datos_pata[(datos_pata['Phase'] > 0.1) & (datos_pata['Phase'] < 0.4)]
    stance_puro = datos_pata[(datos_pata['Phase'] > 0.6) & (datos_pata['Phase'] < 0.9)]
    impacto = datos_pata[(datos_pata['Phase'] >= 0.45) & (datos_pata['Phase'] <= 0.55)]

    print(f"\n--- {nombres_patas[leg_id].upper()} ---")

    if not swing_puro.empty and not stance_puro.empty and not impacto.empty:
        # Promedios en el aire (Swing)
        swing_f_mean = swing_puro['Torque_Femur_Abs'].mean()
        swing_t_mean = swing_puro['Torque_Tibia_Abs'].mean()
        
        # Pico máximo durante la zona de impacto (transición a Stance)
        impacto_f_max = impacto['Torque_Femur_Abs'].max()
        impacto_t_max = impacto['Torque_Tibia_Abs'].max()
        
        # Promedios empujando el suelo (Stance)
        stance_f_mean = stance_puro['Torque_Femur_Abs'].mean()
        stance_t_mean = stance_puro['Torque_Tibia_Abs'].mean()

        print(f"  [AIRE]  Torque base para sostener la pata:")
        print(f"          Fémur: {swing_f_mean:.3f} Nm | Tibia: {swing_t_mean:.3f} Nm")
        
        print(f"  [IMPACTO] Pico máximo al chocar contra el suelo:")
        print(f"          Fémur: {impacto_f_max:.3f} Nm | Tibia: {impacto_t_max:.3f} Nm")

        print(f"  [SUELO] Torque promedio empujando el cuerpo:")
        print(f"          Fémur: {stance_f_mean:.3f} Nm | Tibia: {stance_t_mean:.3f} Nm")

        print(f"  💡 SUGERENCIA DE UMBRAL (Delta = Impacto - Aire):")
        print(f"     > Fémur: Configurar umbral en +{(impacto_f_max - swing_f_mean):.3f} Nm por encima del base.")
        print(f"     > Tibia: Configurar umbral en +{(impacto_t_max - swing_t_mean):.3f} Nm por encima del base.")
    else:
        print("  No hay suficientes datos limpios para analizar esta pata.")

print("="*55 + "\n")

# Mostrar la gráfica interactiva
plt.show()
