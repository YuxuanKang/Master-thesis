TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


INCLUDEPATH +=   /usr/include/
LIBS        += -L/usr/include/
LIBS        += -lCGAL
LIBS        += -lgmp
LIBS        += -lmpfr
LIBS        += -lboost_system
LIBS        += -lglut
LIBS        += -lGLU
LIBS        += -lGL
LIBS        += -lpthread
LIBS        += -L/usr/X11R6/lib
LIBS        += -lm
LIBS        += -lX11
QMAKE_CXXFLAGS += -frounding-math -O3
QMAKE_CXXFLAGS += -O0
QMAKE_CXXFLAGS -= -O1
QMAKE_CXXFLAGS -= -O2
QMAKE_CXXFLAGS -= -O3
QMAKE_CXXFLAGS += -lpthread

SOURCES += \
    main.cpp

HEADERS += \
    objloader.hpp \
    meshoperator.hpp \
    utilities.hpp\
    meshsimplification.hpp\
    ../../../usr/include/CImg.h \
    textureoperator.hpp

