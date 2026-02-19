// File:          test_sensores.cpp
// Date:          2026-02-10
// Description:   Programa de prueba para identificar el mapeo de sensores del e-puck
// Author:        Test Program
//
// INSTRUCCIONES DE USO:
// 1. Ejecutar este controlador en Webots
// 2. Mover el robot cerca de paredes desde diferentes ángulos
// 3. Observar qué sensores se activan (valores altos = cercanos a obstáculo)
// 4. Documentar el mapeo: ps0 = posición?, ps1 = posición?, etc.

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

using namespace webots;

// Color codes para output
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_WHITE   "\033[37m"
#define COLOR_RED     "\033[31m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"

void printColored(const std::string& message, const std::string& color = COLOR_RESET) {
    std::cout << color << message << COLOR_RESET << std::endl;
}

void clearScreen() {
    // Limpiar pantalla (funciona en la mayoría de terminales)
    std::cout << "\033[2J\033[1;1H";
}

void printHeader() {
    printColored("╔════════════════════════════════════════════════════════════════════╗", COLOR_CYAN);
    printColored("║        PROGRAMA DE PRUEBA DE SENSORES E-PUCK                       ║", COLOR_CYAN);
    printColored("║        Identificación de Mapeo de Sensores                         ║", COLOR_CYAN);
    printColored("╚════════════════════════════════════════════════════════════════════╝", COLOR_CYAN);
    std::cout << std::endl;
}

void printInstructions() {
    printColored("INSTRUCCIONES:", COLOR_YELLOW);
    std::cout << "  1. Coloca el robot frente a una pared → observa qué sensores se activan" << std::endl;
    std::cout << "  2. Coloca una pared a la DERECHA del robot → observa qué sensores se activan" << std::endl;
    std::cout << "  3. Coloca una pared a la IZQUIERDA del robot → observa qué sensores se activan" << std::endl;
    std::cout << "  4. Coloca el robot de espaldas a una pared → observa qué sensores se activan" << std::endl;
    std::cout << std::endl;
    printColored("THRESHOLD: Valores > 80 indican detección de pared cercana", COLOR_MAGENTA);
    std::cout << std::endl;
}

void printRobotDiagram() {
    printColored("       DIAGRAMA DEL ROBOT (vista superior):", COLOR_GREEN);
    std::cout << "                  FRENTE" << std::endl;
    std::cout << "                    ↑" << std::endl;
    std::cout << "              ┌─────────┐" << std::endl;
    std::cout << "              │  ● ● ●  │  ← Sensores frontales" << std::endl;
    std::cout << "         IZQ  ●         ●  DER" << std::endl;
    std::cout << "              │         │" << std::endl;
    std::cout << "              │  ● ● ●  │  ← Sensores traseros" << std::endl;
    std::cout << "              └─────────┘" << std::endl;
    std::cout << std::endl;
}

std::string getBarGraph(double value, int maxWidth = 30) {
    const double maxValue = 1000.0; // Valor máximo típico de sensores
    int barLength = static_cast<int>((value / maxValue) * maxWidth);
    if (barLength > maxWidth) barLength = maxWidth;
    
    std::string bar = "";
    for (int i = 0; i < barLength; i++) {
        bar += "█";
    }
    for (int i = barLength; i < maxWidth; i++) {
        bar += "░";
    }
    return bar;
}

std::string getColorForValue(double value, double threshold = 80.0) {
    if (value > threshold * 3) return COLOR_RED;      // Muy cerca
    else if (value > threshold * 1.5) return COLOR_YELLOW; // Cerca
    else if (value > threshold) return COLOR_GREEN;   // Detectado
    else return COLOR_WHITE;                           // Lejos
}

void printSensorData(DistanceSensor* ds[8], int numSensors) {
    const double THRESHOLD = 80.0;
    
    printColored("┌────────────────────────────────────────────────────────────────────┐", COLOR_BLUE);
    printColored("│  VALORES DE SENSORES EN TIEMPO REAL                                │", COLOR_BLUE);
    printColored("└────────────────────────────────────────────────────────────────────┘", COLOR_BLUE);
    
    std::vector<double> values(numSensors);
    std::vector<bool> detected(numSensors);
    
    // Leer valores
    for (int i = 0; i < numSensors; i++) {
        values[i] = ds[i]->getValue();
        detected[i] = values[i] > THRESHOLD;
    }
    
    // Mostrar cada sensor
    for (int i = 0; i < numSensors; i++) {
        std::string color = getColorForValue(values[i], THRESHOLD);
        std::string detectionMark = detected[i] ? " ★ DETECTADO" : "";
        
        std::cout << color;
        std::cout << "  ps" << i << ": " 
                  << std::setw(8) << std::fixed << std::setprecision(2) << values[i]
                  << "  [" << getBarGraph(values[i], 25) << "]"
                  << detectionMark;
        std::cout << COLOR_RESET << std::endl;
    }
    
    std::cout << std::endl;
    
    // Resumen de sensores activos
    std::vector<int> activeSensors;
    for (int i = 0; i < numSensors; i++) {
        if (detected[i]) {
            activeSensors.push_back(i);
        }
    }
    
    if (activeSensors.empty()) {
        printColored("  ► No hay sensores detectando paredes cercanas", COLOR_WHITE);
    } else {
        std::cout << COLOR_YELLOW << "  ► SENSORES ACTIVOS: " << COLOR_RESET;
        for (size_t i = 0; i < activeSensors.size(); i++) {
            std::cout << COLOR_RED << "ps" << activeSensors[i] << COLOR_RESET;
            if (i < activeSensors.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
    }
    
    std::cout << std::endl;
}

void printSuggestions(DistanceSensor* ds[8], int numSensors) {
    const double THRESHOLD = 80.0;
    std::vector<int> activeSensors;
    
    for (int i = 0; i < numSensors; i++) {
        if (ds[i]->getValue() > THRESHOLD) {
            activeSensors.push_back(i);
        }
    }
    
    if (activeSensors.empty()) {
        printColored("💡 SUGERENCIA: Acerca el robot a una pared para ver qué sensores se activan", COLOR_CYAN);
    } else {
        printColored("💡 INTERPRETACIÓN SUGERIDA:", COLOR_CYAN);
        std::cout << "   - Anota la posición actual del robot respecto a las paredes" << std::endl;
        std::cout << "   - Identifica qué sensores (ps0-ps" << (numSensors-1) << ") están activos" << std::endl;
        std::cout << "   - Repite con el robot en otras orientaciones" << std::endl;
    }
    
    std::cout << std::endl;
}

int main(int argc, char **argv) {
    Robot *robot = new Robot();
    int timeStep = static_cast<int>(robot->getBasicTimeStep());
    
    // Intentar inicializar 8 sensores (estándar del e-puck)
    DistanceSensor* ds[8];
    const char* dsNames[8] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
    int numSensors = 0;
    
    // Detectar cuántos sensores tiene el robot
    for (int i = 0; i < 8; i++) {
        ds[i] = robot->getDistanceSensor(dsNames[i]);
        if (ds[i] != nullptr) {
            ds[i]->enable(timeStep);
            numSensors++;
        } else {
            std::cout << "Advertencia: Sensor " << dsNames[i] << " no encontrado" << std::endl;
        }
    }
    
    if (numSensors == 0) {
        printColored("ERROR: No se encontraron sensores de distancia!", COLOR_RED);
        delete robot;
        return 1;
    }
    
    printColored("╔════════════════════════════════════════════════════════════════════╗", COLOR_GREEN);
    std::cout << COLOR_GREEN << "║  Sensores detectados: " << numSensors << " / 8" << std::string(45 - std::to_string(numSensors).length(), ' ') << "║" << COLOR_RESET << std::endl;
    printColored("╚════════════════════════════════════════════════════════════════════╝", COLOR_GREEN);
    std::cout << std::endl;
    
    // Esperar un ciclo para inicializar sensores
    robot->step(timeStep);
    
    int frameCount = 0;
    
    // Loop principal
    while (robot->step(timeStep) != -1) {
        frameCount++;
        
        // Actualizar display cada 10 frames (~100ms típicamente)
        if (frameCount % 10 == 0) {
            clearScreen();
            printHeader();
            printInstructions();
            printSensorData(ds, numSensors);
            printRobotDiagram();
            printSuggestions(ds, numSensors);
            
            printColored("────────────────────────────────────────────────────────────────────", COLOR_BLUE);
            std::cout << "Presiona Ctrl+C en la consola de Webots para detener" << std::endl;
        }
    }
    
    delete robot;
    return 0;
}
