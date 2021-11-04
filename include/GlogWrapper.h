#pragma once
#include <glog/logging.h>
#include <glog/raw_logging.h>

class GlogWrapper{
public:
  GlogWrapper(const char *program);
  ~GlogWrapper();
};