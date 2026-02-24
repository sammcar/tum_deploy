#!/bin/bash

# 1. Limpieza: Cerramos cualquier instancia previa para evitar conflictos en la memoria compartida
sudo pkill -9 imu_publisher 2>/dev/null
sudo pkill -9 contact_publisher 2>/dev/null
sudo pkill -9 tel_viewer 2>/dev/null

echo "🚀 Iniciando ATOM-51..."

# 2. Ejecutar el publicador de IMU en segundo plano (&)
sudo ./imu_publisher &
echo "  [OK] Driver de IMU ejecutándose en segundo plano."

# 3. Ejecutar el estimador de contactos en segundo plano (&)
sudo ./contact_publisher &
echo "  [OK] Estimador de contactos (ZMP) ejecutándose en segundo plano."

# 4. Espera de 1 segundo para dar tiempo a que mmap cree los archivos en /dev/shm
sleep 1

# 5. Ejecutar el controlador de poses en primer plano
echo "  [OK] Iniciando Controlador de Poses..."
echo "------------------------------------------"
sudo ./new_motors/monitor

# 6. Al cerrar el controlador (Ctrl+C), matamos los nodos que quedaron de fondo
sudo pkill -9 imu_publisher
sudo pkill -9 contact_publisher
echo "Sistemas cerrados correctamente."
