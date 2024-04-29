#include "/home/tianhu/TO/src/towr/towr_ros/third_party/glog/src/glog/logging.h"

int main(int argc, char** argv) {
  FLAGS_logtostderr = 1;
  google::InitGoogleLogging(argv[0]);
  LOG(INFO) << "Hello world!";
  CHECK(false) << "Test stacktrace printer";
  return 0;
}
