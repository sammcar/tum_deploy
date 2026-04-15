// tum_quad_terminal.cpp - TUM - Control de Cuadrúpedo (Terminal, Sin GUI)
//
// Dependencias Bazel (todas ya en :mech):
//   @boost//:beast, @boost//:asio
// Gamepad:
//   raw read() sobre /dev/input/eventN — solo linux/input.h
//   sin libevdev, sin linkopts extra
// JSON:
//   Construcción manual (send) + parser mínimo de strings (receive).
//   No se usa boost::json ni jsoncpp para evitar conflicto entre
//   boost del sistema y boost del workspace de Bazel.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// Boost Beast / Asio — igual que web_server.cc
#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

// Gamepad Linux (raw evdev — solo linux/input.h, sin libevdev)
#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace beast = boost::beast;
namespace ws    = beast::websocket;
namespace net   = boost::asio;
using tcp       = net::ip::tcp;

// ============================================================
// CONSTANTES (idénticas al Python)
// ============================================================
static constexpr double CMD_MAX_RATE_Y    = 0.2;
static constexpr double CMD_MAX_RATE_Z    = M_PI / 3.0;
static constexpr double CMD_MAX_POSE_YAW  = M_PI / 18.0;
static constexpr double CMD_MAX_POSE_PITCH= M_PI / 18.0;
static constexpr double CMD_MAX_POSE_ROLL = M_PI / 18.0;
static constexpr double CMD_MAX_POSE_X    = 0.05;
static constexpr double CMD_MAX_POSE_Y_V  = 0.02;
static constexpr double CMD_MAX_POSE_Z    = 0.06;

// ============================================================
// ANSI
// ============================================================
#define ANSI_RESET   "\033[0m"
#define ANSI_BOLD    "\033[1m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_RED     "\033[31m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_BLUE    "\033[34m"
#define ANSI_GRAY    "\033[90m"
#define ANSI_WHITE   "\033[37m"
#define ANSI_CLR     "\033[2J\033[H"

// ============================================================
// DEBUG — si es false, no se imprime nada en terminal
// ============================================================
static constexpr bool DEBUG = true;

// ============================================================
// ESTRUCTURAS
// ============================================================
struct JoyState {
    double lx=0, ly=0, rx=0, ry=0;
    uint32_t buttons=0;
    bool hat_up=false, hat_down=false, hat_left=false, hat_right=false;
    double lt=0, rt=0;
};

struct RobotState {
    std::string mode         = "unknown";
    double      voltage      = 0.0;
    double      max_temp     = 0.0;
    int         joints_count = 0;
    std::string fault        = "";
    bool        connected    = false;
    bool        routine_done = false;
    double      cycle_ms     = 0.0;
    int         missing      = 0;
};

// ============================================================
// ESTADO GLOBAL
// ============================================================
struct AppState {
    JoyState joy_state;
    uint32_t prev_buttons = 0;
    struct { bool up=false,down=false,left=false,right=false; } prev_hat;

    std::string mode           = "stop";
    double      max_speed      = 0.5;
    std::string body_pose_mode = "velocity";

    struct Routine { std::string id; std::string label; };
    std::vector<Routine> routines = {
        {"kFlexion",    "Flexiones"},
        {"kBaile",      "Baile"},
        {"kSentarse",   "Sentarse"},
        {"kLevantarse", "Levantarse"},
    };
    int         routine_index     = 0;
    std::string routine_active_id = "kFlexion";
    bool        routine_active    = false;
    bool        routine_was_active= false;
    bool        routine_holding   = false;

    bool enable_strafe = false;
    bool always_step   = false;
    bool record_data   = false;

    RobotState robot_state;

    std::chrono::steady_clock::time_point last_trigger_time =
        std::chrono::steady_clock::now();

    std::atomic<bool> running{true};

    // joy_mtx: solo protege joy_state
    // mtx:     lógica, modo, rutinas, telemetría
    std::mutex joy_mtx;
    std::mutex mtx;
};

static AppState g;

// ============================================================
// UTILIDADES DE TERMINAL
// ============================================================
static void clear_screen() { std::cout << ANSI_CLR; }

static std::string bar(double value, double max_val, int width,
                       const char* color) {
    int filled = static_cast<int>(
        std::max(0.0, std::min(1.0, value / max_val)) * width);
    std::string s = color;
    s += "[";
    for (int i = 0; i < width; ++i) s += (i < filled) ? "=" : "-";
    s += "]" ANSI_RESET;
    return s;
}

// ============================================================
// JSON — construcción manual
// ============================================================

// Formatea un double sin notación científica con precisión suficiente
static std::string jd(double v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << v;
    return ss.str();
}

static std::string build_command_json() {
    // Copiar estado bajo sus respectivos mutexes
    JoyState j;
    { std::lock_guard<std::mutex> jlk(g.joy_mtx); j = g.joy_state; }

    std::lock_guard<std::mutex> lock(g.mtx);

    // Construir v_R, w_R y pose según body_pose_mode
    std::string v_R, w_R, extra;

    if (g.body_pose_mode == "translation") {
        v_R = "[0,0,0]";
        w_R = "[0,0,0]";
        extra = ",\"rest\":{\"offset_RB\":{"
                "\"translation\":["
                + jd(j.ly * CMD_MAX_POSE_X)   + ","
                + jd(j.lx * CMD_MAX_POSE_Y_V) + ","
                + jd(-j.ry * CMD_MAX_POSE_Z)  +
                "],\"so3\":{\"w\":1,\"x\":0,\"y\":0,\"z\":0}}}";

    } else if (g.body_pose_mode == "rotation") {
        v_R = "[0,0,0]";
        w_R = "[0,0,0]";
        extra = ",\"rest\":{\"offset_RB\":{"
                "\"translation\":[0,0,0],"
                "\"so3\":{"
                "\"w\":1,"
                "\"x\":" + jd(j.lx * CMD_MAX_POSE_ROLL)  + ","
                "\"y\":" + jd(j.ly * CMD_MAX_POSE_PITCH) + ","
                "\"z\":" + jd(j.rx * CMD_MAX_POSE_YAW)   +
                "}}}";

    } else {
        double vx = j.ly * g.max_speed;
        double vy = g.enable_strafe ? j.lx * CMD_MAX_RATE_Y : 0.0;
        double wz = j.rx * CMD_MAX_RATE_Z;
        v_R = "[" + jd(vx) + "," + jd(vy) + ",0]";
        w_R = "[0,0," + jd(wz) + "]";
    }

    // Detectar movimiento
    bool moving = false;
    if (g.body_pose_mode == "velocity") {
        double vx = j.ly * g.max_speed;
        double vy = g.enable_strafe ? j.lx * CMD_MAX_RATE_Y : 0.0;
        double wz = j.rx * CMD_MAX_RATE_Z;
        moving = std::fabs(vx) > 0.025 || std::fabs(vy) > 0.025 ||
                 std::fabs(wz) > 0.025;
    }

    // mode_map
    std::string mode_str;
    if      (g.mode == "off")     mode_str = "stopped";
    else if (g.mode == "stop")    mode_str = "zero_velocity";
    else if (g.mode == "idle")    mode_str = "rest";
    else if (g.mode == "walk")    mode_str = (moving || g.always_step) ? "walk" : "rest";
    else if (g.mode == "pronk")   mode_str = "jump";
    else if (g.mode == "situp")   mode_str = "situp";
    else if (g.mode == "routine") mode_str = "routine";
    else                          mode_str = "zero_velocity";

    std::string log_str = g.record_data ? "enable" : "disable";

    // Campos opcionales
    if (g.mode == "pronk")
        extra += ",\"jump\":{\"acceleration\":8,\"repeat\":true}";
    if (g.mode == "situp")
        extra += ",\"situp\":{}";
    if (g.mode == "routine" && g.routine_active)
        extra += ",\"routine\":{\"routine_id\":\"" + g.routine_active_id + "\"}";

    return "{\"command\":{"
           "\"mode\":\"" + mode_str + "\","
           "\"v_R\":"    + v_R      + ","
           "\"w_R\":"    + w_R      + ","
           "\"log\":\""  + log_str  + "\""
           + extra +
           "}}";
}

// ============================================================
// JSON — parser mínimo para telemetría entrante
// ============================================================
// Extrae el valor de una clave string simple: "key":"value"
// ============================================================
// JSON — parser contextual
// Estructura del mensaje (según QuadrupedState + web_control.h):
// {
//   "mode": "rest",               <- root
//   "fault": "",                  <- root
//   "missing_replies": 0,         <- root
//   "timing": {"cycle_s": 0.001}, <- root
//   "state": {
//     "joints": [{"temperature_C":45, "voltage":8.2, ...}, ...],
//     "robot":  {"voltage": 38.5, ...},   <- batería aquí
//     "routine":{"done": false, ...}
//   }
// }
// ============================================================

// Busca "key" y avanza hasta el valor (salta espacios y ':')
static size_t json_find_val(const std::string& s, const std::string& key,
                            size_t from = 0) {
    auto pos = s.find("\"" + key + "\"", from);
    if (pos == std::string::npos) return std::string::npos;
    pos += key.size() + 2;
    while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t' || s[pos] == '\n' || s[pos] == '\r')) ++pos;
    if (pos >= s.size() || s[pos] != ':') return std::string::npos;
    ++pos;
    while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t' || s[pos] == '\n' || s[pos] == '\r')) ++pos;
    return pos;
}

// Extrae el contenido entre llaves/corchetes de "key" : {...} o "key" : [...]
static std::string json_section(const std::string& s, const std::string& key) {
    auto pos = json_find_val(s, key);
    if (pos == std::string::npos || pos >= s.size()) return "";
    char open  = s[pos];
    char close = (open == '{') ? '}' : (open == '[') ? ']' : 0;
    if (!close) return "";
    int depth = 0;
    auto start = pos;
    for (size_t i = pos; i < s.size(); ++i) {
        if (s[i] == open)  ++depth;
        if (s[i] == close) { if (--depth == 0) return s.substr(start, i - start + 1); }
    }
    return "";
}

// Extrae string: "key" : "value"
static std::string json_str(const std::string& s, const std::string& key) {
    auto pos = json_find_val(s, key);
    if (pos == std::string::npos || pos >= s.size() || s[pos] != '"') return "";
    ++pos;
    auto end = s.find('"', pos);
    if (end == std::string::npos) return "";
    return s.substr(pos, end - pos);
}

// Extrae número: "key" : number
static double json_num(const std::string& s, const std::string& key,
                       double def = 0.0) {
    auto pos = json_find_val(s, key);
    if (pos == std::string::npos) return def;
    try { return std::stod(s.substr(pos)); }
    catch (...) { return def; }
}

// Extrae bool: "key" : true/false
static bool json_bool(const std::string& s, const std::string& key) {
    auto pos = json_find_val(s, key);
    if (pos == std::string::npos) return false;
    return s.size() > pos + 3 && s.substr(pos, 4) == "true";
}

// Temperatura máxima y conteo de joints iterando sobre el array
static void json_parse_joints(const std::string& joints_arr,
                              double& max_temp, int& count) {
    max_temp = 0.0; count = 0;
    size_t pos = 0;
    while (true) {
        auto vpos = json_find_val(joints_arr, "temperature_C", pos);
        if (vpos == std::string::npos) break;
        try { max_temp = std::max(max_temp, std::stod(joints_arr.substr(vpos))); }
        catch (...) {}
        ++count;
        pos = vpos + 1;
    }
}

static void process_status(const std::string& raw) {
    std::lock_guard<std::mutex> lock(g.mtx);
    auto& rs = g.robot_state;

    // Campos en el root
    rs.mode    = json_str(raw, "mode");
    if (rs.mode.empty()) rs.mode = "unknown";
    rs.fault   = json_str(raw, "fault");
    rs.cycle_ms = json_num(raw, "cycle_s") * 1000.0;
    rs.missing  = static_cast<int>(json_num(raw, "missing_replies"));
    rs.connected = true;

    // state.robot.voltage — batería
    const std::string state_sec  = json_section(raw, "state");
    const std::string robot_sec  = json_section(state_sec, "robot");
    rs.voltage = json_num(robot_sec, "voltage");

    // state.joints — temperatura y conteo
    const std::string joints_sec = json_section(state_sec, "joints");
    json_parse_joints(joints_sec, rs.max_temp, rs.joints_count);

    // state.routine.done
    const std::string routine_sec = json_section(state_sec, "routine");
    rs.routine_done = json_bool(routine_sec, "done");

    // Lógica de rutinas (igual que antes)
    static const std::vector<std::string> HOLD_ROUTINES =
        {"kSentarse", "kLevantarse"};
    bool is_hold = std::find(HOLD_ROUTINES.begin(), HOLD_ROUTINES.end(),
                             g.routine_active_id) != HOLD_ROUTINES.end();

    if (rs.mode == "routine" && g.routine_active) {
        g.routine_was_active = true;
        if (is_hold && !g.routine_holding)
            g.routine_holding = true;
    }
    if (g.routine_active && g.routine_was_active &&
        !g.routine_holding && rs.routine_done) {
        g.routine_active     = false;
        g.routine_was_active = false;
        g.routine_holding    = false;
        g.mode               = "idle";
    }
}

// ============================================================
// GAMEPAD (evdev)
// ============================================================
static void gamepad_thread_fn() {
    // Deteccion via ioctl — sin libevdev, compila en x86 y ARM cross-compiler
    std::string dev_path;
    for (int i = 0; i < 32; ++i) {
        std::string p = "/dev/input/event" + std::to_string(i);
        int fd = open(p.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;

        uint8_t evbits[EV_MAX / 8 + 1] = {};
        ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits);
        bool has_abs = evbits[EV_ABS / 8] & (1 << (EV_ABS % 8));
        bool has_key = evbits[EV_KEY / 8] & (1 << (EV_KEY % 8));

        if (has_abs && has_key) {
            uint8_t keybits[KEY_MAX / 8 + 1] = {};
            ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits);
            if (keybits[BTN_GAMEPAD / 8] & (1 << (BTN_GAMEPAD % 8))) {
                dev_path = p;
                close(fd);
                break;
            }
        }
        close(fd);
    }

    if (dev_path.empty()) {
        if (DEBUG) std::cout << ANSI_YELLOW "Gamepad no encontrado.\n" ANSI_RESET;
        return;
    }

    int fd = open(dev_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) return;

    if (DEBUG) {
        char name[256] = "desconocido";
        ioctl(fd, EVIOCGNAME(sizeof(name)), name);
        std::cout << ANSI_GREEN "Gamepad: " << name << ANSI_RESET "\n";
    }

    static const std::map<int,int> bit_map = {
        {BTN_SOUTH,0},{BTN_A,0},{BTN_EAST,1},{BTN_B,1},
        {BTN_NORTH,2},{BTN_X,2},{BTN_WEST,3},{BTN_Y,3},
        {BTN_TL,4},{BTN_TR,5},{BTN_TR2,6},{BTN_TL2,7},
    };

    while (g.running) {
        struct input_event ev;
        ssize_t n = read(fd, &ev, sizeof(ev));

        if (n < 0) {
            if (errno == EAGAIN) {
                std::this_thread::sleep_for(std::chrono::milliseconds(8));
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (n < (ssize_t)sizeof(ev)) continue;

        std::lock_guard<std::mutex> lock(g.joy_mtx);
        auto& j = g.joy_state;

        if (ev.type == EV_ABS) {
            switch (ev.code) {
                case ABS_X:     j.lx =  ev.value / 32768.0; break;
                case ABS_Y:     j.ly = -ev.value / 32768.0; break;
                case ABS_RX:    j.rx =  ev.value / 32768.0; break;
                case ABS_RY:    j.ry = -ev.value / 32768.0; break;
                case ABS_HAT0X:
                    j.hat_left  = (ev.value == -1);
                    j.hat_right = (ev.value ==  1);
                    break;
                case ABS_HAT0Y:
                    j.hat_up   = (ev.value == -1);
                    j.hat_down = (ev.value ==  1);
                    break;
                case ABS_Z: case ABS_BRAKE:
                    j.lt = ev.value / 255.0; break;
                case ABS_RZ: case ABS_GAS:
                    j.rt = ev.value / 255.0; break;
            }
        } else if (ev.type == EV_KEY) {
            auto it = bit_map.find(ev.code);
            if (it != bit_map.end()) {
                if (ev.value) j.buttons |=  (1u << it->second);
                else          j.buttons &= ~(1u << it->second);
            }
        }
    }
    close(fd);
}


// ============================================================
// LÓGICA DE CONTROL (asumen g.mtx ya tomado)
// ============================================================
static void set_mode(const std::string& mode) {
    static const std::vector<std::string> safe =
        {"idle","walk","pronk","situp","stop","off","routine"};
    if (std::find(safe.begin(), safe.end(), mode) == safe.end()) return;
    if (mode == "stop" && g.mode != "idle" && g.mode != "off") return;
    if (mode == g.mode) return;
    g.mode = mode;
    if (mode != "routine") {
        g.routine_active  = false;
        g.routine_holding = false;
    }
}

static void select_routine(int idx) {
    if (g.routine_active && !g.routine_holding) return;
    g.routine_index = idx;
}

static void launch_routine() {
    if (g.mode != "routine") set_mode("routine");
    g.routine_active      = true;
    g.routine_holding     = false;
    g.routine_was_active  = false;
    g.routine_active_id   = g.routines[g.routine_index].id;
}

static void cycle_body_pose_mode() {
    if      (g.body_pose_mode == "velocity")    g.body_pose_mode = "translation";
    else if (g.body_pose_mode == "translation") g.body_pose_mode = "rotation";
    else if (g.body_pose_mode == "rotation")    g.body_pose_mode = "routine";
    else {
        g.body_pose_mode = "velocity";
        if (g.mode == "routine" && !g.routine_holding)
            set_mode("idle");
    }
}

// ============================================================
// PROCESS INPUT (20 Hz)
// ============================================================
static void process_input() {
    JoyState j;
    { std::lock_guard<std::mutex> jlk(g.joy_mtx); j = g.joy_state; }
    std::lock_guard<std::mutex> lock(g.mtx);

    uint32_t cur  = j.buttons;
    uint32_t prev = g.prev_buttons;
    auto pressed  = [&](int bit) {
        return (cur & (1u << bit)) && !(prev & (1u << bit));
    };

    bool lt_now = j.lt > 0.5 || (cur & (1u << 7));
    bool rt_now = j.rt > 0.5 || (cur & (1u << 6));

    if (pressed(4)) cycle_body_pose_mode();              // LB
    if (pressed(3)) {                                    // Y
        if (g.body_pose_mode == "routine") launch_routine();
        else g.enable_strafe = !g.enable_strafe;
    }
    if (pressed(0)) g.always_step = !g.always_step;     // A
    if (pressed(1)) g.record_data = !g.record_data;     // B
    if (pressed(2)) set_mode("stop");                   // X
    if (pressed(5)) set_mode("off");                    // RB

    g.prev_buttons = cur;

    bool p_up=g.prev_hat.up, p_down=g.prev_hat.down;
    bool p_left=g.prev_hat.left, p_right=g.prev_hat.right;

    if (g.body_pose_mode == "routine") {
        if (j.hat_left  && !p_left)
            select_routine((g.routine_index - 1 + (int)g.routines.size()) %
                           (int)g.routines.size());
        if (j.hat_right && !p_right)
            select_routine((g.routine_index + 1) % (int)g.routines.size());
    } else {
        if (j.hat_up    && !p_up)    set_mode("idle");
        if (j.hat_down  && !p_down)  set_mode("situp");
        if (j.hat_right && !p_right) set_mode("walk");
        if (j.hat_left  && !p_left)  set_mode("pronk");
    }
    g.prev_hat = {j.hat_up, j.hat_down, j.hat_left, j.hat_right};

    auto now = std::chrono::steady_clock::now();
    auto dt  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now - g.last_trigger_time).count();
    if (lt_now && dt > 200) {
        g.max_speed = std::max(0.1, g.max_speed - 0.1);
        g.last_trigger_time = now;
    }
    if (rt_now && dt > 200) {
        g.max_speed = std::min(2.0, g.max_speed + 0.1);
        g.last_trigger_time = now;
    }
}

// ============================================================
// RENDER TERMINAL
// ============================================================
static void render_terminal() {
    JoyState j;
    { std::lock_guard<std::mutex> jlk(g.joy_mtx); j = g.joy_state; }
    std::lock_guard<std::mutex> lock(g.mtx);
    auto& rs = g.robot_state;

    clear_screen();
    std::cout
        << ANSI_BOLD ANSI_WHITE
        << "╔══════════════════════════════════════════════════════╗\n"
        << "║      PLATAFORMA CUADRUPEDA TUM  --  Terminal Control ║\n"
        << "╚══════════════════════════════════════════════════════╝\n"
        << ANSI_RESET;

    // Conexión & batería
    double batt_pct = std::max(0.0, std::min(100.0,
        100.0 * (rs.voltage - 36.0) / (42.0 - 36.0)));
    const char* bc = batt_pct > 60 ? ANSI_GREEN :
                     batt_pct > 30 ? ANSI_YELLOW : ANSI_RED;
    std::string conn = rs.connected
        ? (ANSI_GREEN "● CONECTADO" ANSI_RESET)
        : (ANSI_RED   "○ SIN CONEXION" ANSI_RESET);
    std::cout << "  " << conn
              << "   Bateria: " << bc
              << std::fixed << std::setprecision(1)
              << rs.voltage << "V (" << static_cast<int>(batt_pct) << "%)"
              << ANSI_RESET "  " << bar(batt_pct, 100, 20, bc) << "\n\n";

    // Modos
    std::string mc, ml;
    if      (g.body_pose_mode == "translation")
        { mc = ANSI_YELLOW;  ml = "POSTURA: TRASLACION"; }
    else if (g.body_pose_mode == "rotation")
        { mc = ANSI_MAGENTA; ml = "POSTURA: ROTACION"; }
    else if (g.body_pose_mode == "routine")
        { mc = ANSI_YELLOW;  ml = "RUTINA: " + g.routines[g.routine_index].label; }
    else
        { mc = ANSI_CYAN;    ml = "CONTROL DE VELOCIDAD"; }

    std::cout << "  Modo control : " << mc << ANSI_BOLD << ml << ANSI_RESET << "\n"
              << "  Modo local   : " << ANSI_BOLD << g.mode << ANSI_RESET << "\n";
    if (rs.connected)
        std::cout << "  Modo robot   : " << ANSI_CYAN << rs.mode << ANSI_RESET "\n";
    else
        std::cout << "  Modo robot   : " << ANSI_GRAY "sin conexion" ANSI_RESET "\n";
    std::cout << "\n";

    // Velocidad
    double vel_mag = std::hypot(j.ly * g.max_speed,
                                g.enable_strafe ? j.lx * CMD_MAX_RATE_Y : 0.0);
    double wz_deg  = (j.rx * CMD_MAX_RATE_Z) * 180.0 / M_PI;
    std::cout << "  Velocidad    : " << ANSI_CYAN ANSI_BOLD
              << std::setprecision(2) << vel_mag << " m/s" ANSI_RESET
              << "  " << bar(vel_mag, g.max_speed, 20, ANSI_CYAN) << "\n"
              << "  Limite vel.  : " ANSI_BOLD
              << std::setprecision(1) << g.max_speed << " m/s" ANSI_RESET "\n";
    if (std::fabs(wz_deg) > 1.0)
        std::cout << "  Giro (Yaw)   : " << ANSI_YELLOW ANSI_BOLD
                  << std::setw(6) << std::setprecision(1) << wz_deg
                  << " grados/s" ANSI_RESET "\n";
    std::cout << "\n";

    // Opciones
    auto flag = [](bool v, const char* lbl) {
        return std::string(v ? ANSI_GREEN "[X] " : ANSI_GRAY "[ ] ") + lbl + ANSI_RESET;
    };
    std::cout << "  " << flag(g.enable_strafe, "Strafe (Y)")
              << "  " << flag(g.always_step,   "Caminata (A)")
              << "  " << flag(g.record_data,   "Grabar (B)") << "\n\n";

    // Rutinas
    std::cout << "  Rutinas: ";
    for (int i = 0; i < (int)g.routines.size(); ++i) {
        bool sel  = (i == g.routine_index);
        bool runn = (g.routine_active && g.routine_active_id == g.routines[i].id);
        if      (runn) std::cout << ANSI_YELLOW ANSI_BOLD;
        else if (sel)  std::cout << ANSI_CYAN   ANSI_BOLD;
        else           std::cout << ANSI_GRAY;
        std::cout << "[" << g.routines[i].label << "]" ANSI_RESET "  ";
    }
    std::string rstat = g.routine_holding  ? (ANSI_BLUE  "Pose mantenida" ANSI_RESET) :
                        g.routine_active   ? (ANSI_YELLOW "Ejecutando..."  ANSI_RESET) :
                                             (ANSI_GRAY   "Sin rutina"     ANSI_RESET);
    std::cout << "\n  Estado rutina: " << rstat << "\n\n";

    // Telemetría
    std::cout
        << "  +----- Telemetria ------------------------------------------+\n"
        << "  |  MODO:    " << std::setw(14) << std::left << rs.mode
        << "  TEMP: " << std::setprecision(1) << rs.max_temp << "C"
        << "              |\n"
        << "  |  BATERIA: " << std::setw(6) << rs.voltage << "V"
        << "         MOTORES: " << rs.joints_count << "/12"
        << "              |\n"
        << "  |  CICLO:   " << std::setprecision(1) << rs.cycle_ms << "ms"
        << "         PERDIDOS: " << rs.missing
        << "              |\n";
    if (!rs.fault.empty())
        std::cout << "  |  " ANSI_RED ANSI_BOLD "FALLO: " << rs.fault
                  << ANSI_RESET "  |\n";
    std::cout
        << "  +-----------------------------------------------------------+\n\n"
        << ANSI_GRAY
        << "  LB: Ciclar modo | Cruceta: Modos | X: Detener | RB: Apagar\n"
        << "  LT/RT: Velocidad | Y: Strafe/Rutina | A: Caminata | Ctrl+C: Salir\n"
        << ANSI_RESET;
    std::cout.flush();
}

// ============================================================
// WEBSOCKET (boost::beast, hilo dedicado)
// ============================================================
static void websocket_thread_fn(const std::string& robot_ip) {
    while (g.running) {
        try {
            net::io_context ioc;
            tcp::resolver resolver(ioc);
            ws::stream<beast::tcp_stream> stream(ioc);

            auto results = resolver.resolve(robot_ip, "4778");
            beast::get_lowest_layer(stream).connect(results);
            // Sin timeout — igual que el Python: bloqueante pero sin límite artificial
            beast::get_lowest_layer(stream).expires_never();
            stream.handshake(robot_ip, "/control");

            {
                std::lock_guard<std::mutex> lk(g.mtx);
                g.robot_state.connected = true;
            }
            if (DEBUG) std::cout << ANSI_GREEN "Conectado a " << robot_ip
                                   << ":4778\n" ANSI_RESET;

            beast::flat_buffer recv_buf;

            while (g.running) {
                // 1. Enviar comando — igual que Python: send cada iteración
                std::string cmd = build_command_json();
                stream.write(net::buffer(cmd));

                // 2. Leer telemetría — el servidor responde a cada comando
                //    Bloqueante sin timeout: cuando llega, llega. Sin polling.
                stream.read(recv_buf);
                process_status(beast::buffers_to_string(recv_buf.data()));
                recv_buf.consume(recv_buf.size());
            }
            stream.close(ws::close_code::normal);

        } catch (const std::exception& e) {
            {
                std::lock_guard<std::mutex> lk(g.mtx);
                g.robot_state.connected = false;
            }
            if (g.running && DEBUG)
                std::cout << ANSI_YELLOW "Conexion perdida: " << e.what()
                          << ". Reintentando en 2s...\n" ANSI_RESET;
        }
        if (g.running)
            std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// ============================================================
// MAIN
// ============================================================
static void signal_handler(int) { g.running = false; }

int main(int argc, char* argv[]) {
    std::string robot_ip = "192.168.16.47";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if      (arg == "--local")             robot_ip = "localhost";
        else if (arg == "--ip" && i+1 < argc)  robot_ip = argv[++i];
    }

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    if (DEBUG) std::cout << "\n" ANSI_BOLD ANSI_WHITE
              << "================================\n"
              << "   TUM - Control Cuadrupedo\n"
              << "================================\n" ANSI_RESET "\n"
              << "Conectando a " << robot_ip << ":4778\n"
              << "Presiona Ctrl+C para salir\n\n";

    std::thread gp_thread(gamepad_thread_fn);
    std::thread ws_thread(websocket_thread_fn, robot_ip);

    // Hilo de render separado — no bloquea el loop de control
    std::thread render_thread([]() {
        auto last_debug = std::chrono::steady_clock::now();
        while (g.running) {
            if (DEBUG) {
                auto now = std::chrono::steady_clock::now();
                auto dt  = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - last_debug).count();
                if (dt >= 2000) {
                    render_terminal();
                    last_debug = now;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    // Hilo principal: solo process_input a 20 Hz
    auto next = std::chrono::steady_clock::now();
    while (g.running) {
        next += std::chrono::milliseconds(50);
        process_input();
        std::this_thread::sleep_until(next);
    }

    std::cout << "\nCerrando TUM...\n";
    gp_thread.join();
    ws_thread.join();
    render_thread.join();
    std::cout << "Limpieza completa\n";
    return 0;
}