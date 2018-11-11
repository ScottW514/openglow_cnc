cmake_minimum_required( VERSION 3.12 )
project( openglow_cnc C )

set(CMAKE_C_STANDARD 99)

set( XENOMAI_C_FLAGS " -D_GNU_SOURCE -D_REENTRANT -fasynchronous-unwind-tables -D__MERCURY__ -D_GNU_SOURCE " )
set( XENOMAI_LD_FLAGS " -Wl,--no-as-needed -Wl,--wrap=main -Wl,--dynamic-list=dynlist.ld " )

set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENOMAI_C_FLAGS} -Wformat -Wall -g -O0 " )

set( CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 ")

if (${TARGET_BUILD}) # Set this to indicate we are building in Yocto for the target arch
    message(STATUS "Yocto target build deetected.")
    add_compile_definitions(TARGET_BUILD)
else()
    message(STATUS "Dev host build detected.")
    set( STAGING_DIR_TARGET "")
endif()
set( SOURCE_FILES src/main.c src/common.h src/openglow-cnc.h src/cli/console.c src/cli/console.h src/cli/socket.c src/cli/socket.h src/config.h src/system/system.c src/system/system.h src/motion/gcode.c src/motion/gcode.h src/motion/motion_control.c src/motion/motion_control.h src/motion/planner.c src/motion/planner.h src/motion/segment.c src/motion/segment.h src/cli/messages.c src/cli/messages.h src/hardware/limits.c src/hardware/limits.h src/hardware/laser.c src/hardware/laser.h src/cli/cli.h src/cli/cli.c src/motion/grbl_glue.c src/motion/grbl_glue.h src/hardware/stepgen.c src/hardware/stepgen.h src/hardware/openglow.c src/hardware/openglow.h src/hardware/step_drv.c src/hardware/step_drv.h src/hardware/switches.c src/hardware/switches.h src/system/settings.c src/system/settings.h src/system/fsm.c src/system/fsm.h src/hardware/hardware.c src/hardware/hardware.h src/motion/motion.c src/motion/motion.h)

add_executable( openglow_cnc ${SOURCE_FILES} )

target_link_libraries( openglow_cnc -L${STAGING_DIR_TARGET}/usr/xenomai/lib/ -pthread -lalchemy -lcopperplate -lmercury -lrt -lm ${STAGING_DIR_TARGET}/usr/xenomai/lib/xenomai/bootstrap.o )

set_target_properties( openglow_cnc PROPERTIES LINK_FLAGS " -Wl,--no-as-needed -Wl,--wrap=main -Wl,--dynamic-list=${STAGING_DIR_TARGET}/usr/xenomai/lib/dynlist.ld" )

include_directories( ${STAGING_DIR_TARGET}/usr/include/xenomai ${STAGING_DIR_TARGET}/usr/include/xenomai/mercury/ )

link_directories( ${STAGING_DIR_TARGET}/usr/xenomai/lib/ )

install( TARGETS openglow_cnc DESTINATION /usr/bin )
