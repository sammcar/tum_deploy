import os
import ollama

def verificar_ollama(modelo):
    """Verifica que el servidor de Ollama esté corriendo y el modelo exista."""
    try:
        modelos_locales = ollama.list()
    except Exception as e:
        print("\n[ERROR CRÍTICO] No se pudo conectar con Ollama.")
        return False

    try:
        nombres_modelos = [m['model'] for m in modelos_locales.get('models', [])]
    except AttributeError:
        nombres_modelos = [getattr(m, 'model', '') for m in modelos_locales.get('models', [])]
    
    if modelo not in nombres_modelos:
        print(f"\n[ERROR] El modelo '{modelo}' no está instalado.")
        return False
        
    return True

def calentar_modelo(modelo):
    """Fuerza a Ollama a cargar el modelo en RAM/VRAM."""
    print(f"Calentando el modelo '{modelo}'...")
    try:
        ollama.chat(model=modelo, messages=[{'role': 'user', 'content': 'OK'}])
    except Exception:
        pass

def resumir_codigo(codigo, nombre_archivo, modelo):
    """Envía el código al LLM para obtener un resumen ejecutivo estructurado."""
    
    # Prompt estricto enfocado en resúmenes ejecutivos limpios
    # Prompt estricto enfocado en resúmenes ejecutivos y análisis de variables de control
    system_prompt = f"""
    Eres un Arquitecto de Software y Experto en Control de Sistemas Embebidos (Robótica). Tu tarea es analizar el archivo '{nombre_archivo}' y redactar un Resumen Técnico estructurado.
    
    REGLAS ESTRICTAS:
    1. Propósito General: Describe de forma directa y concisa (máximo 2 párrafos) la responsabilidad principal del archivo.
    
    2. ANÁLISIS DE PARÁMETROS CRÍTICOS (CRÍTICO): Busca explícita y minuciosamente el uso de las siguientes 4 variables en el código: 
       'rb_filter_constant_Hz', 'lr_acceleration', 'lr_alpha_rad_s2', 'terrain_filter_s'.
       
       - Si ALGUNA de estas variables aparece en el código: Crea una sección obligatoria llamada "### Análisis de Parámetros de Sintonización". Para CADA variable que encuentres, detalla usando viñetas:
         * **Uso:** Para qué se utiliza exactamente en este archivo (ej. filtro paso bajo, límite de saturación).
         * **Efecto al AUMENTAR:** Qué impacto físico, matemático o de estabilidad tendría en el robot si el valor sube.
         * **Efecto al DISMINUIR:** Qué impacto físico, matemático o de estabilidad tendría en el robot si el valor baja.
       
       - Si NINGUNA de estas variables se usa: Omite la sección anterior por completo y añade solo esta nota: "Nota: Este archivo no gestiona los parámetros críticos de sintonización dinámica." BAJO NINGUNA CIRCUNSTANCIA inventes que las variables existen si no están en el código.
       
    3. Componentes Clave: Lista las clases, funciones o estructuras de datos más importantes usando viñetas (bullet points). Deja siempre un salto de línea limpio (\\n) antes de empezar la lista.
    
    4. Sanitización: NO uses caracteres especiales que dependan de UTF-8 estricto (ej. cambia ≥ por >= y µ por u).
    
    5. Cero Charla: NO devuelvas el código original. NO uses introducciones como "Aquí tienes el resumen". Empieza directamente con el contenido.
    """

    try:
        respuesta = ollama.chat(model=modelo, messages=[
            {'role': 'system', 'content': system_prompt},
            {'role': 'user', 'content': codigo}
        ])
        return respuesta['message']['content']  .strip()
    except Exception as e:
        print(f"\n  [ERROR] Fallo al resumir {nombre_archivo}: {e}")
        return "No se pudo generar el resumen debido a un error de conexión con Ollama."

def generar_resumen_proyecto(carpeta_objetivo, modelo="llama3.1:latest"):
    """Recorre la carpeta, resume cada archivo y crea un MD global."""
    print("=== INICIANDO AGENTE DE RESUMEN EJECUTIVO ===")
    
    if not os.path.exists(carpeta_objetivo):
        print(f"[ERROR] La ruta '{carpeta_objetivo}' no existe.")
        return

    if not verificar_ollama(modelo):
        return
    calentar_modelo(modelo)

    # Archivo de salida en la misma carpeta objetivo
    ruta_md = os.path.join(carpeta_objetivo, "resumen_del_proyecto.md")
    extensiones_validas = ['.h', '.hpp', '.c', '.cpp', '.py']
    
    # Inicializamos el archivo Markdown con un título principal
    with open(ruta_md, 'w', encoding='utf-8') as f_out:
        f_out.write("# Resumen Ejecutivo del Proyecto\n\n")
        f_out.write("> Documento generado automáticamente por IA.\n\n")
        f_out.write("---\n\n")

    archivos = sorted(os.listdir(carpeta_objetivo))
    archivos_procesados = 0

    for archivo in archivos:
        ruta_archivo = os.path.join(carpeta_objetivo, archivo)
        
        if os.path.isfile(ruta_archivo):
            _, extension = os.path.splitext(archivo)
            
            if extension.lower() in extensiones_validas:
                print(f"📄 Analizando y resumiendo: {archivo}...")
                
                # Leer el código fuente
                with open(ruta_archivo, 'r', encoding='utf-8') as f_in:
                    codigo = f_in.read()
                
                # Obtener el resumen del LLM
                resumen = resumir_codigo(codigo, archivo, modelo)
                
                # Escribir directamente en el archivo MD global
                with open(ruta_md, 'a', encoding='utf-8') as f_out:
                    f_out.write(f"## Archivo: `{archivo}`\n\n")
                    f_out.write(f"{resumen}\n\n")
                    f_out.write("---\n\n")
                
                archivos_procesados += 1

    print("=" * 45)
    print(f"=== PROCESO FINALIZADO ===")
    print(f"Se resumieron {archivos_procesados} archivos.")
    print(f"El documento final está listo en: {ruta_md}")

if __name__ == "__main__":
    # =================================================================
    # CONFIGURACIÓN: Cambia esta ruta por la carpeta que quieras resumir
    # =================================================================
    RUTA_DE_TUS_ARCHIVOS = "/home/sebas/Documents/Github/tum_deploy/documentacion/tum_deploy_bazel/mech" 
    MODELO_A_USAR = "llama3.1:latest" # Puedes cambiarlo a qwen3.5:latest si prefieres
    
    # Ejecutamos el agente
    generar_resumen_proyecto(RUTA_DE_TUS_ARCHIVOS, MODELO_A_USAR)