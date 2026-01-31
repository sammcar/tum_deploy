#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>
#include <map>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/joystick.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>

// --- CONFIGURACIÓN ---
#define JS_DEVICE "/dev/input/js0"
#define SHM_NAME "/tum_shared"
#define MEM_SIZE 4096
#define TARGET_IP "192.168.103.141" 
#define TARGET_PORT 5005
#define LOOP_RATE_MS 20 

// --- ESTRUCTURA BINARIA (PACKET) ---
// Esta estructura debe ser IDÉNTICA en Python
// __attribute__((packed)) asegura que no haya espacios vacíos entre bytes
struct __attribute__((packed)) TelemetryPacket {
    float vx, vy, wz;              // 12 bytes
    float body_x, body_y, body_z;  // 12 bytes
    float roll, pitch, yaw;        // 12 bytes
    float freq;                    // 4 bytes
    float default_z;               // 4 bytes (CALIBRACIÓN AUTOMÁTICA)
    int8_t mode;                   // 1 byte  (0:STAND, 1:CRAWL, 2:TROT)
    int8_t submode;                // 1 byte  (0:TRANS, 1:ROT)
    bool use_imu;                  // 1 byte
    // Total aprox: 47 bytes (vs 400 del JSON)
};

// --- ESTRUCTURAS INTERNAS (Lógica) ---
struct RobotState {
    double vx=0, vy=0, wz=0;
    double body_x=0, body_y=0, body_z=0.22; // Default Z aquí
    double body_roll=0, body_pitch=0, body_yaw=0;
    std::string gait_mode = "STAND";
    std::string submode = "TRANSLATION";
    double trot_freq = 2.6;
    bool use_imu = false;
};

struct ControlContext {
    std::string mode = "STAND";
    std::string stand_submode = "TRANSLATION";
    RobotState current;
    RobotState target;
    double lerp_factor = 0.1;
    std::map<int, double> axes;
    std::map<int, int> buttons;
    std::map<int, int> prev_buttons;
};

// Mantenemos JSON SOLO para la Shared Memory (Local)
std::string serialize_for_shm(const RobotState& s) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "{\"vx\":" << s.vx << ",\"vy\":" << s.vy << ",\"wz\":" << s.wz 
       << ",\"gait_mode\":\"" << s.gait_mode << "\"}"; 
    // (Resumido para SHM, puedes poner el JSON completo si otros procesos lo usan)
    return ss.str();
}

double deadzone(double val) { return (std::abs(val) > 0.08) ? val : 0.0; }

int main() {
    std::cout << "--- TUM CONTROLLER (BINARY UDP) ---" << std::endl;

    int js_fd = open(JS_DEVICE, O_RDONLY | O_NONBLOCK);
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, MEM_SIZE);
    char* shared_mem = (char*)mmap(0, MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(TARGET_PORT);
    servaddr.sin_addr.s_addr = inet_addr(TARGET_IP);

    ControlContext ctx;
    struct js_event js;

    while (true) {
        auto start = std::chrono::steady_clock::now();

        // 1. INPUT
        while (read(js_fd, &js, sizeof(js)) > 0) {
            if (js.type & JS_EVENT_BUTTON) ctx.buttons[js.number] = js.value;
            else if (js.type & JS_EVENT_AXIS) ctx.axes[js.number] = js.value / 32767.0;
        }
        auto pressed = [&](int b) { 
            bool p = ctx.buttons[b] && !ctx.prev_buttons[b]; 
            ctx.prev_buttons[b] = ctx.buttons[b]; return p; 
        };

        // 2. LÓGICA (Simplificada para brevedad, misma lógica que antes)
        if (pressed(0) && ctx.mode == "STAND") ctx.stand_submode = (ctx.stand_submode == "TRANSLATION") ? "ROTATION" : "TRANSLATION";
        if (pressed(1)) ctx.mode = "STAND";
        if (pressed(2)) ctx.mode = "CRAWL";
        if (pressed(3)) ctx.mode = "TROT";
        if (pressed(4)) ctx.current.trot_freq = std::max(1.0, ctx.current.trot_freq - 0.1);
        if (pressed(5)) ctx.current.trot_freq = std::min(4.0, ctx.current.trot_freq + 0.1);
        if (pressed(7)) ctx.current.use_imu = !ctx.current.use_imu;
        
        bool reset_L = pressed(9);
        bool reset_R = pressed(10);
        if(reset_L) { ctx.target.vy=0; ctx.target.body_y=0; ctx.target.body_pitch=0; }
        if(reset_R) { ctx.target.vx=0; ctx.target.wz=0; ctx.target.body_x=0; ctx.target.body_z=0.22; ctx.target.body_roll=0; ctx.target.body_yaw=0; }

        double lx = deadzone(ctx.axes[0]), ly = deadzone(ctx.axes[1]);
        double rx = deadzone(ctx.axes[3]), ry = deadzone(ctx.axes[4]);

        if (!reset_L && !reset_R) {
            if (ctx.mode != "STAND") {
                ctx.target.vx = -ly * 0.5; ctx.target.vy = lx * 0.3; ctx.target.wz = -rx * 1.5; ctx.target.body_z = 0.22;
            } else {
                if (ctx.stand_submode == "TRANSLATION") {
                    ctx.target.body_x = -ly * 0.08; ctx.target.body_y = -lx * 0.06; 
                    ctx.target.body_z = 0.22 + (-ry * 0.09); // Altura dinámica
                } else {
                    ctx.target.body_roll = lx * 25.0; ctx.target.body_pitch = -ly * 20.0; ctx.target.body_yaw = rx * 15.0;
                }
            }
        }

        // LERP
        double f = ctx.lerp_factor;
        ctx.current.vx += (ctx.target.vx - ctx.current.vx) * f;
        ctx.current.vy += (ctx.target.vy - ctx.current.vy) * f;
        ctx.current.wz += (ctx.target.wz - ctx.current.wz) * f;
        ctx.current.body_z += (ctx.target.body_z - ctx.current.body_z) * f;
        ctx.current.body_x += (ctx.target.body_x - ctx.current.body_x) * f;
        ctx.current.body_y += (ctx.target.body_y - ctx.current.body_y) * f;
        ctx.current.body_roll += (ctx.target.body_roll - ctx.current.body_roll) * f;
        ctx.current.body_pitch += (ctx.target.body_pitch - ctx.current.body_pitch) * f;
        ctx.current.body_yaw += (ctx.target.body_yaw - ctx.current.body_yaw) * f;
        ctx.current.gait_mode = ctx.mode;
        ctx.current.submode = ctx.stand_submode;

        // 3. PREPARAR PAQUETE BINARIO (AQUÍ ESTÁ LA MAGIA)
        TelemetryPacket packet;
        packet.vx = (float)ctx.current.vx;
        packet.vy = (float)ctx.current.vy;
        packet.wz = (float)ctx.current.wz;
        
        packet.body_x = (float)ctx.current.body_x;
        packet.body_y = (float)ctx.current.body_y;
        packet.body_z = (float)ctx.current.body_z;
        
        packet.roll = (float)ctx.current.body_roll;
        packet.pitch = (float)ctx.current.body_pitch;
        packet.yaw = (float)ctx.current.body_yaw;
        
        packet.freq = (float)ctx.current.trot_freq;
        packet.default_z = 0.22f; // <--- ENVIAMOS LA CALIBRACIÓN
        
        // Convertir Strings a Enteros para envío rápido
        packet.mode = (ctx.mode == "STAND") ? 0 : (ctx.mode == "CRAWL" ? 1 : 2);
        packet.submode = (ctx.stand_submode == "TRANSLATION") ? 0 : 1;
        packet.use_imu = ctx.current.use_imu;

        // 4. ENVIAR BINARIO (UDP)
        sendto(sockfd, &packet, sizeof(packet), MSG_CONFIRM, (const struct sockaddr *)&servaddr, sizeof(servaddr));
        memcpy(shared_mem, &packet, sizeof(TelemetryPacket));

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        if (elapsed < LOOP_RATE_MS) std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_RATE_MS - elapsed));
    }
    return 0;
}
