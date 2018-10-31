TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    SpaceDSL/source/jpleph.cpp \
    SpaceDSL/source/nrlmsise00.cpp \
    SpaceDSL/source/nrlmsise00_data.cpp \
    SpaceDSL/source/soapC.cpp \
    SpaceDSL/source/soapClient.cpp \
    SpaceDSL/source/SpAtmosphere.cpp \
    SpaceDSL/source/SpCoordSystem.cpp \
    SpaceDSL/source/SpCZMLScript.cpp \
    SpaceDSL/source/SpEnvironment.cpp \
    SpaceDSL/source/SpGravity.cpp \
    SpaceDSL/source/SpIntegration.cpp \
    SpaceDSL/source/SpInterpolation.cpp \
    SpaceDSL/source/SpJplEph.cpp \
    SpaceDSL/source/SpMath.cpp \
    SpaceDSL/source/SpMission.cpp \
    SpaceDSL/source/SpOptimize.cpp \
    SpaceDSL/source/SpOrbitParam.cpp \
    SpaceDSL/source/SpOrbitPredict.cpp \
    SpaceDSL/source/SpPerturbation.cpp \
    SpaceDSL/source/SpPropagator.cpp \
    SpaceDSL/source/SpRightFunction.cpp \
    SpaceDSL/source/SpSpaceVehicle.cpp \
    SpaceDSL/source/SpThread.cpp \
    SpaceDSL/source/SpTimeSystem.cpp \
    SpaceDSL/source/SpUtils.cpp \
    SpaceDSL/source/stdsoap2.cpp

HEADERS += \
    SpaceDSL/Include/SpaceDSL.h \
    SpaceDSL/Include/SpaceDSL_Global.h \
    SpaceDSL/Include/SpAtmosphere.h \
    SpaceDSL/Include/SpConst.h \
    SpaceDSL/Include/SpCoordSystem.h \
    SpaceDSL/Include/SpCZMLScript.h \
    SpaceDSL/Include/SpEnvironment.h \
    SpaceDSL/Include/SpGravity.h \
    SpaceDSL/Include/SpIntegration.h \
    SpaceDSL/Include/SpInterpolation.h \
    SpaceDSL/Include/SpJplEph.h \
    SpaceDSL/Include/SpMath.h \
    SpaceDSL/Include/SpMission.h \
    SpaceDSL/Include/SpOptimize.h \
    SpaceDSL/Include/SpOrbitParam.h \
    SpaceDSL/Include/SpOrbitPredict.h \
    SpaceDSL/Include/SpPerturbation.h \
    SpaceDSL/Include/SpPropagator.h \
    SpaceDSL/Include/SpRightFunction.h \
    SpaceDSL/Include/SpSpaceVehicle.h \
    SpaceDSL/Include/SpThread.h \
    SpaceDSL/Include/SpTimeSystem.h \
    SpaceDSL/Include/SpUtils.h

# mast configure by personal

LIBS += -lpthread #Multithreading

INCLUDEPATH += ./Dependence  \ #Eigen Libray

