OPSYS=$(shell uname)
PLATFORM=$(shell uname -p)
ARCH=.$(PLATFORM)

CXX:=ccache $(CXX)

CXXFLAGS+=-std=c++17 -DROOT_DIR=\\"$(shell pwd ..)/\\"

CPPFLAGS+=$(LOCAL_CFLAGS)
LDFLAGS+=$(LOCAL_LDFLAGS)

# Подключаем заголовки
CPPFLAGS+=-I./include -I/usr/include/eigen3 -I$(GITMAN_DIR)/CTopPRM/include -I$(GITMAN_DIR)/PMM/include

# Линкуем библиотеки
LDFLAGS+=-lpthread -lyaml-cpp -llog4cxx

# Оптимизация компиляции
CXXFLAGS+= -O3 -march=native
