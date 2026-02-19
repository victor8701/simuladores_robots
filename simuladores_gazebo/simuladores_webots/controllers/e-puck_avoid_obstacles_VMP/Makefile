# =========================================================================
# Makefile para controlador Webots C++
# =========================================================================

# Rutas
WEBOTS_HOME = /usr/local/webots

# Nombre del ejecutable (DEBE coincidir con el nombre del controlador en Webots)
TARGET = e-puck_avoid_obstacles_VMP

# Archivo fuente (cambia si tu archivo tiene otro nombre)
SOURCE = e-puck_avoid_obstacles_VMP.cpp

# Compilador
CXX = g++

# Flags de compilación
CXXFLAGS = -I$(WEBOTS_HOME)/include/controller/cpp \
           -std=c++11 \
           -Wall \
           -Wextra \
           -O2

# Flags de enlazado
LDFLAGS = -L$(WEBOTS_HOME)/lib/controller
LIBS = -lCppController

# Regla principal
all: $(TARGET)

# Regla para compilar
$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE) $(LDFLAGS) $(LIBS)

# Limpiar
clean:
	rm -f $(TARGET)
	rm -f *.o

# Recompilar desde cero
rebuild: clean all

.PHONY: all clean rebuild
