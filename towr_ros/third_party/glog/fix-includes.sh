#!/bin/sh
rm -rf src/package_config_unittest
find -name '*test*' -delete
for i in src/*.h src/*.cc src/base/* src/glog/*; do
  sed -i 's@#include "base/@#include "third_party/glog/src/base/@' $i
  sed -i 's@#include "config.h"@#include "third_party/glog/src/config.h"@' $i
  sed -i 's@#include "demangle.h"@#include "third_party/glog/src/demangle.h"@' $i
  sed -i 's@#include "stacktrace.h"@#include "third_party/glog/src/stacktrace.h"@' $i
  sed -i 's@#include "symbolize.h"@#include "third_party/glog/src/symbolize.h"@' $i
  sed -i 's@#include "utilities.h"@#include "third_party/glog/src/utilities.h"@' $i
  sed -i 's@#include <glog/\([^>]*\)>@#include "third_party/glog/src/glog/\1"@' $i
done
cmake -P cmake/ConfigureHeaders.cmake
find -name '*.in' -delete
