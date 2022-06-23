cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_foc_controller)

target_sources(app PRIVATE src/foc_controller.c)
target_sources(app PRIVATE src/foc_svm.c)
target_sources(app PRIVATE src/main.c)
