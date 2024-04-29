project (glog LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Threads REQUIRED)
find_package(Unwind REQUIRED)

add_library(glog STATIC
  src/base/commandlineflags.h
  src/base/googleinit.h
  src/base/mutex.h
  src/demangle.cc
  src/demangle.h
  src/logging.cc
  src/raw_logging.cc
  src/signalhandler.cc
  src/symbolize.cc
  src/symbolize.h
  src/utilities.cc
  src/utilities.h
  src/vlog_is_on.cc
)
target_compile_definitions(glog
  PUBLIC
    GLOG_CUSTOM_PREFIX_SUPPORT
    GLOG_NO_SYMBOLIZE_DETECTION
)
target_include_directories(glog
  PUBLIC
    ${PENGUIN_INCLUDE_DIR}
    # Not using SYSTEM include to override use of <glog/logging.h> in system libraries
    ${PENGUIN_INCLUDE_DIR}/third_party/glog/src
)
target_link_libraries(glog
  PRIVATE
    Threads::Threads
    unwind::unwind
)
set_target_properties(glog
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    VISIBILITY_INLINES_HIDDEN ON
    CXX_VISIBILITY_PRESET hidden
    C_VISIBILITY_PRESET hidden
)

add_executable(example_glog_app
  example_glog_app.cc
)
target_link_libraries(example_glog_app
  PRIVATE
    glog
)
