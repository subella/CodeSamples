ifeq ($(CILKSAN),1)
CXXFLAGS += -fsanitize=cilk
LDFLAGS += -fsanitize=cilk
endif

ifeq ($(CILKSCALE),1)
CFLAGS += -fcsi
LDFLAGS += -lclang_rt.cilkscale-x86_64
endif
