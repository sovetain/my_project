include Mk/libs_flags.mk

SRC_DIR = src
BUILD_DIR = build
GITMAN_DIR = gitman_sources
INCLUDE_DIRS = -Iinclude -I$(GITMAN_DIR)/CTopPRM/include -I$(GITMAN_DIR)/PMM/include
LIB_DIRS = -L$(GITMAN_DIR)/CTopPRM/lib -L$(GITMAN_DIR)/PMM/lib

SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:%.cpp=$(BUILD_DIR)/%.o)

LIBS = -llog4cxx -lyaml-cpp -lpthread -lstdc++

TARGET = bin/planner

$(shell mkdir -p $(BUILD_DIR))
$(shell mkdir -p bin)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(LIB_DIRS) $(OBJS) -o $(TARGET) $(LIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)/*.o $(TARGET)

rebuild: clean $(TARGET)

update_gitman:
	gitman install
