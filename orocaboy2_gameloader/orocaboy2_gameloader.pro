TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.c \
    uart_win.c \
    util_win.c \
    util.c \
    cmd.c \
    loader.c \
    download.c

HEADERS += \
    def.h \
    uart.h \
    util.h \
    cmd.h \
    loader.h \
    download.h
