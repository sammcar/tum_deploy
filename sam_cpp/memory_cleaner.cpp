#include <iostream>
#include <cstring>      // para std::memset
#include <fcntl.h>      // para constantes O_RDWR
#include <sys/mman.h>   // para shm_open, mmap
#include <unistd.h>     // para close
#include <sys/stat.h>   // para modos de archivo

// ============================================================
// DEFINICIÓN DE ESTRUCTURA (Debe ser idéntica al main)
// ============================================================
#pragma pack(push, 1)
struct SharedData {
    double angles[4][3];        // [Pata][Motor]
    double velocities[4][3];    
    double desired_accel[4][3]; // [X, Y, Z] Feedforward
    double kp_scale[4][3];      
    double kd_scale[4][3];      
    bool is_stance[4];          
    bool is_walking;      
};
#pragma pack(pop)

int main() {
    const char* shm_name = "/rex_shm";

    // 1. Abrir la memoria compartida existente
    // No usamos O_CREAT porque solo queremos limpiar si ya existe.
    int shm_fd = shm_open(shm_name, O_RDWR, 0666);

    if (shm_fd == -1) {
        std::cout << "⚠️ No se encontró memoria compartida activa ('/rex_shm')." << std::endl;
        std::cout << "   (El sistema ya está limpio o no se ha iniciado nunca)." << std::endl;
        return 0;
    }

    // 2. Mapear la memoria
    SharedData* shm_ptr = static_cast<SharedData*>(
        mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0)
    );

    if (shm_ptr == MAP_FAILED) {
        std::cerr << "❌ Error al acceder a la memoria (mmap failed)." << std::endl;
        close(shm_fd);
        return 1;
    }

    // 3. LIMPIEZA TOTAL
    std::cout << "🧹 Limpiando memoria compartida..." << std::endl;
    
    // Pone todo a 0 (ángulos, velocidades, aceleraciones, escalas, bools)
    std::memset(shm_ptr, 0, sizeof(SharedData));
    
    // Aseguramos explícitamente el flag de seguridad
    shm_ptr->is_walking = false;

    // 4. Sincronizar y Cerrar
    // msync asegura que los cambios se escriban físicamente ahora mismo
    msync(shm_ptr, sizeof(SharedData), MS_SYNC);

    munmap(shm_ptr, sizeof(SharedData));
    close(shm_fd);

    std::cout << "✅ Memoria '/rex_shm' reseteada a CERO con éxito." << std::endl;
    std::cout << "   El robot es seguro para iniciar." << std::endl;

    return 0;
}
