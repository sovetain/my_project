uniq = $(if $1,$(firstword $1) $(call uniq,$(filter-out $(firstword $1),$1)))

CXXFLAGS:=$(call uniq,$(CXXFLAGS))

clean_make: clean $(TARGET)

bin: $(TARGET)

TARGET_OBJS=$(addprefix $(BUILD_DIR)/,$(OBJS))

$(OBJS): %.o: src/%.cpp
	echo "Compiling $<"
	$(CXX) -c $< $(CXXFLAGS) $(CPPFLAGS) -o $(BUILD_DIR)/$@

$(TARGET): create_directories $(OBJS)
	echo "Building $(TARGET)"
	$(CXX) $(CXXFLAGS) -o $@ $(TARGET_OBJS) $(LDFLAGS)

create_directories:
	echo "Creating build directory"
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(TARGET)
	rm -rf $(BUILD_DIR)/*
