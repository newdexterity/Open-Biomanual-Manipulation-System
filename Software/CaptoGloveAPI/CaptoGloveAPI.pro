QT += core bluetooth
QT -= gui

# For final version without debug
CONFIG += CAPTOGLOVEAPI_LIBRARY


CONFIG += c+11
CONFIG += qt

TARGET = CaptoGloveAPI

CAPTOGLOVEAPI_LIBRARY {
    TEMPLATE=lib
    CONFIG += static
    DEFINES += CAPTOGLOVEAPI_LIBRARY
}else{
    CONFIG += console
    SOURCES += main.cpp
}

CONFIG += console

DEFINES += LINUX

DEFINES += PROJECT_PATH=\"\\\"$${_PRO_FILE_PWD_}/\\\"\"

SOURCES = captogloveapi.cpp\
          deviceinfo.cpp \
          logger.cpp \
          serviceinfo.cpp \
          characteristicinfo.cpp \
          main.cpp

HEADERS = captogloveapi.h \
          deviceinfo.h \
          logger.h \
          serviceinfo.h \
          characteristicinfo.h

# Protobuffer compiler
message("Generating protocol buffer classes from .proto files.")
message("Protoc version:" $$VERSION)

PROTO_DECL_PATH = protobuffers
PROTO_IMPL_PATH = proto_impl

system(mkdir proto_impl)
for(p, $$list($$files($${PROTO_DECL_PATH}\*.proto))){
    message("Generating protobuffer:" $$basename(p))
    VERSION = $$system("protoc --version")
    system(protoc --cpp_out=$${PROTO_IMPL_PATH} --proto_path=$${PROTO_DECL_PATH} $$basename(p))
    PROTO_IMPL_FILE = $${PROTO_IMPL_PATH}/$$basename(p)
    SOURCES +=$$replace(PROTO_IMPL_FILE, .proto, .pb.cc)
    HEADERS +=$$replace(PROTO_IMPL_FILE, .proto, .pb.h)
}

# Linking
unix{

    LIBS += -pthread

    # Include protobuf
    LIBS += -L"/home/developer/protobuf/build" -lprotobuf

}

QMAKE_CXXFLAGS += -std=gnu++0x -pthread
QMAKE_CFLAGS += -std=gnu++0x -pthread


DISTFILES += \
    protobuffers/captoglove_v1.proto \
    config.ini
 
