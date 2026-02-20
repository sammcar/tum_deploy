#!/bin/bash

# 1. Limpieza: Cerramos cualquier instancia previa para evitar conflictos en la memoria compartida
sudo pkill -9 imu_publisher 2>/dev/null
sudo pkill -9 tel_viewer 2>/dev/null

echo "🚀 Iniciando ATOM-51..."

# 2. Ejecutar el publicador de IMU en segundo plano (&)
sudo ./imu_publisher &
echo "  [OK] Driver de IMU ejecutándose en segundo plano."

# 3. Espera de 1 segundo
sleep 1

# 4. Ejecutar el controlador de poses en primer plano
echo "  [OK] Iniciando Controlador de Poses..."
echo "------------------------------------------"
sudo ./new_motors/monitor

# 5. Al cerrar el controlador (Ctrl+C), matamos la IMU que quedó de fondo
sudo pkill -9 imu_publisher
echo "Sistemas cerrados correctamente."
