# Makefile for test_sensores controller

### Generic Makefile.include for Webots controllers
WEBOTS_HOME ?= /usr/local/webots

# Include path for controller libraries
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

# Add linker flags to resolve GLIBC compatibility issues
LDFLAGS += -ldl -lpthread

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
