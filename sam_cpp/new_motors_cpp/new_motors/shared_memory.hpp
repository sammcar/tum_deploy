#pragma once
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cstdint>

struct CommandData
{
    double angles[4][3]; // [Patas][Joints]
    double velocities[4][3];
    double desired_accel[4][3];
    double kp_scale[4][3];
    double kd_scale[4][3];
    double transition_time;
    bool is_stance[4];
    bool is_walking;

};

// --- ESTRUCTURA DE TELEMETRÍA (Escribe C++ -> Lee Python) ---
struct TelemetryData
{
    double measured_angles[4][3];
    double measured_velocities[4][3];
    double measured_torques[4][3];
    double temperature[4][3];
    uint64_t timestamp_us;
    bool fault_code[4]; // Ejemplo: si una pata falló
};

class MemoryManager
{
private:
    // Helper para abrir/crear memoria
    template <typename T>
    void open_shm(const char *name, int &fd, T *&ptr)
    {
        fd = shm_open(name, O_RDWR | O_CREAT, 0666);
        if (fd == -1)
        {
            std::cerr << "[SHM] Error abriendo " << name << std::endl;
            return;
        }
        struct stat shm_stat;
        fstat(fd, &shm_stat);

        if (shm_stat.st_size != sizeof(T))
        {
            ftruncate(fd, sizeof(T));
        }

        void *map = mmap(0, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (map == MAP_FAILED)
        {
            std::cerr << "[SHM] Error mapeando " << name << std::endl;
            return;
        }
        ptr = static_cast<T *>(map);

        // Limpiar si es nuevo
        if (shm_stat.st_size == 0 || shm_stat.st_size != sizeof(T))
        {
            std::memset(ptr, 0, sizeof(T));
        }
    }

public:
    const char *cmd_name = "/rex_cmd";
    const char *tel_name = "/rex_tel";

    int fd_cmd = -1;
    int fd_tel = -1;

    CommandData *cmd = nullptr;
    TelemetryData *tel = nullptr;

    MemoryManager()
    {
        open_shm<CommandData>(cmd_name, fd_cmd, cmd);
        open_shm<TelemetryData>(tel_name, fd_tel, tel);
    }

    ~MemoryManager()
    {
        if (cmd)
            munmap(cmd, sizeof(CommandData));
        if (tel)
            munmap(tel, sizeof(TelemetryData));
        if (fd_cmd != -1)
            close(fd_cmd);
        if (fd_tel != -1)
            close(fd_tel);
    }

    bool is_valid() { return cmd != nullptr && tel != nullptr; }
};
