#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <string>
#include <limits>
#include <atomic>
#include <csignal>

// Declaración de la variable global (se definirá físicamente en main.cpp)
extern std::atomic<bool> g_running;

// Declaración del manejador de señales (se definirá físicamente en main.cpp)
void SignalHandler(int signum);

template <typename T>
T pedir_dato(std::string mensaje) {
    T valor; std::cout << ">> " << mensaje << ": ";
    while (!(std::cin >> valor)) {
        std::cin.clear(); std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "❌ Error. " << mensaje << ": ";
    }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    return valor;
}

#endif // UTILS_HPP