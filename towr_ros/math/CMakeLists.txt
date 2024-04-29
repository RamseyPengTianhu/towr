# Copyright 2021 Tencent. All rights reserved.
add_library(math STATIC
  constants.h
  cross_product.h
)
target_include_directories(math
  PUBLIC
    ${PENGUIN_INCLUDE_DIR}
)
target_link_libraries(math
  PUBLIC
    eigen
)
set_target_properties(math PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(math PROPERTIES LINKER_LANGUAGE CXX)