###########################################################################
## Makefile generated for component 'detect2'. 
## 
## Makefile     : detect2_rtw.mk
## Generated on : Sun Apr 03 14:20:42 2022
## Final product: .detect2.a
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# MODELLIB                Static library target

PRODUCT_NAME              = detect2
MAKEFILE                  = detect2_rtw.mk
MATLAB_ROOT               = E:\Software\MATLAB\R2021b
MATLAB_BIN                = E:\Software\MATLAB\R2021b\bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)glnxa64
START_DIR                 = E:\Projects\Matlab\InnovateFPGA\image20220329
TGT_FCN_LIB               = ISO_C++11
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = ......
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
MODELLIB                  = detect2.a

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Cadence Xcelium (64-bit Linux)
# Supported Version(s):    1.0
# ToolchainInfo Version:   2021b
# Specification Revision:  1.0
# 
TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: Cadence Xcelium (64-bit Linux) C Compiler
CC = gcc

# Linker: Cadence Xcelium (64-bit Linux) Linker
LD = gcc

# C++ Compiler: Cadence Xcelium (64-bit Linux) GNU C++ Compiler
CPP = g++

# C++ Linker: Cadence Xcelium (64-bit Linux) GNU C++ Linker
CPP_LD = g++

# Archiver: Cadence Xcelium (64-bit Linux) GNU Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: Cadence Xcelium (64-bit Linux)
MAKE = echo "### Successfully generated all binary outputs."


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              =
C_OUTPUT_FLAG       =
LDDEBUG             =
OUTPUT_FLAG         =
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  =
ECHO                =
MV                  =
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              =
CFLAGS               =
CPPFLAGS             =
CPP_LDFLAGS          =
CPP_SHAREDLIB_LDFLAGS  =
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              =
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           =  $(MAKEFILE)
SHAREDLIB_LDFLAGS    =



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = .detect2.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -ccflags$(START_DIR)\codegen\lib\detect2 -ccflags$(START_DIR) -ccflags$(MATLAB_ROOT)\extern\include

INCLUDES = 

###########################################################################
## DEFINES
###########################################################################

DEFINES_CUSTOM = 
DEFINES_STANDARD = MODEL=detect2

DEFINES =  

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)codegenlibdetect2detect2_data.cpp $(START_DIR)codegenlibdetect2detect2_initialize.cpp $(START_DIR)codegenlibdetect2detect2_terminate.cpp $(START_DIR)codegenlibdetect2detect2.cpp $(START_DIR)codegenlibdetect2inPainting.cpp $(START_DIR)codegenlibdetect2imfilter.cpp $(START_DIR)codegenlibdetect2imbinarize.cpp $(START_DIR)codegenlibdetect2adaptthresh.cpp $(START_DIR)codegenlibdetect2bwlabel.cpp $(START_DIR)codegenlibdetect2regionprops.cpp $(START_DIR)codegenlibdetect2bwconncomp.cpp $(START_DIR)codegenlibdetect2useConstantDim.cpp $(START_DIR)codegenlibdetect2imreconstruct.cpp

ALL_SRCS = 

###########################################################################
## OBJECTS
###########################################################################

OBJS = detect2_data.o detect2_initialize.o detect2_terminate.o detect2.o inPainting.o imfilter.o imbinarize.o adaptthresh.o bwlabel.o regionprops.o bwconncomp.o useConstantDim.o imreconstruct.o

ALL_OBJS = 

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += 

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += 

###########################################################################
## INLINED COMMANDS
###########################################################################

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	 "### Successfully generated all binary outputs."


build : prebuild 


prebuild : 


download : 


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

 :  
	 "### Creating static library "" ..."
	    $(subst ,/,$(subst ,/,))
	 "### Created: "


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	   "$@" $(subst ,/,"$<")


%.o : %.cpp
	  -o "$@" $(subst ,/,"$<")


%.o : $(START_DIR)codegenlibdetect2%.c
	   "$@" $(subst ,/,"$<")


%.o : $(START_DIR)codegenlibdetect2%.cpp
	  -o "$@" $(subst ,/,"$<")


%.o : $(START_DIR)%.c
	   "$@" $(subst ,/,"$<")


%.o : $(START_DIR)%.cpp
	  -o "$@" $(subst ,/,"$<")


detect2_data.o : $(START_DIR)codegenlibdetect2detect2_data.cpp
	  -o "$@" $(subst ,/,"$<")


detect2_initialize.o : $(START_DIR)codegenlibdetect2detect2_initialize.cpp
	  -o "$@" $(subst ,/,"$<")


detect2_terminate.o : $(START_DIR)codegenlibdetect2detect2_terminate.cpp
	  -o "$@" $(subst ,/,"$<")


detect2.o : $(START_DIR)codegenlibdetect2detect2.cpp
	  -o "$@" $(subst ,/,"$<")


inPainting.o : $(START_DIR)codegenlibdetect2inPainting.cpp
	  -o "$@" $(subst ,/,"$<")


imfilter.o : $(START_DIR)codegenlibdetect2imfilter.cpp
	  -o "$@" $(subst ,/,"$<")


imbinarize.o : $(START_DIR)codegenlibdetect2imbinarize.cpp
	  -o "$@" $(subst ,/,"$<")


adaptthresh.o : $(START_DIR)codegenlibdetect2adaptthresh.cpp
	  -o "$@" $(subst ,/,"$<")


bwlabel.o : $(START_DIR)codegenlibdetect2bwlabel.cpp
	  -o "$@" $(subst ,/,"$<")


regionprops.o : $(START_DIR)codegenlibdetect2regionprops.cpp
	  -o "$@" $(subst ,/,"$<")


bwconncomp.o : $(START_DIR)codegenlibdetect2bwconncomp.cpp
	  -o "$@" $(subst ,/,"$<")


useConstantDim.o : $(START_DIR)codegenlibdetect2useConstantDim.cpp
	  -o "$@" $(subst ,/,"$<")


imreconstruct.o : $(START_DIR)codegenlibdetect2imreconstruct.cpp
	  -o "$@" $(subst ,/,"$<")


###########################################################################
## DEPENDENCIES
###########################################################################

 : rtw_proj.tmw 


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	 "### PRODUCT = "
	 "### PRODUCT_TYPE = "
	 "### BUILD_TYPE = "
	 "### INCLUDES = "
	 "### DEFINES = "
	 "### ALL_SRCS = "
	 "### ALL_OBJS = "
	 "### LIBS = "
	 "### MODELREF_LIBS = "
	 "### SYSTEM_LIBS = "
	 "### TOOLCHAIN_LIBS = "
	 "### CFLAGS = "
	 "### LDFLAGS = "
	 "### SHAREDLIB_LDFLAGS = "
	 "### CPPFLAGS = "
	 "### CPP_LDFLAGS = "
	 "### CPP_SHAREDLIB_LDFLAGS = "
	 "### ARFLAGS = "
	 "### MEX_CFLAGS = "
	 "### MEX_CPPFLAGS = "
	 "### MEX_LDFLAGS = "
	 "### MEX_CPPLDFLAGS = "
	 "### DOWNLOAD_FLAGS = "
	 "### EXECUTE_FLAGS = "
	 "### MAKE_FLAGS = "


clean : 
	 "### Deleting all derived files..."
	 $(subst /,\,)
	 $(subst /,\,)
	 "### Deleted all derived files."


