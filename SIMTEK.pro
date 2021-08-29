TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        Calculations.cpp \
        main.cpp \
        shapeobjects.cpp

QMAKE_CXXFLAGS += -std=c++0x -pthread
LIBS += -pthread

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv4

HEADERS += \
    Calculations.h \
    shapeobjects.h
