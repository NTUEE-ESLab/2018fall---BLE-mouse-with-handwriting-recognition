TARGET1=mouse
OBJECT1=mouse.o
CC=gcc
CXX=g++
LD_FLAGS += -lpthread -lbluetooth
C_FLAGS=
# end of user configuration
all: $(TARGET1)
$(TARGET1) : $(OBJECT1)
	$(CC) $^ -o $@ $(C_FLAGS)
%.o : %.c
	$(CC) $(C_FLAGS) -c $<
%.o : %.cpp
	$(CXX) $(C_FLAGS) -c $<
.PHONY: clean
clean :
	-rm -f $(OBJECT1) $(TARGET1)
