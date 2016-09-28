#ifndef CREEK_MODEL_LOADER_HPP
#define CREEK_MODEL_LOADER_HPP

#ifdef USE_CNOID_MODEL

#include <cnoid/BodyLoader>
#include "cnoidBody.hpp"
namespace creek
{
  bool loadBody(creek::BodyPtr body, const std::string& path)
  {
    cnoid::BodyLoader bl;
    return bl.load(*body, path);
  }
}

#elif defined USE_HRP_MODEL

#include <hrpModel/ModelLoaderUtil.h>
#include "hrpBody.hpp"
namespace creek
{
  bool loadBody(creek::BodyPtr body, const std::string& path)
  {
    int argc(1);
    char* argv[1] = {"a"};
    return hrp::loadBodyFromModelLoader(body, path.c_str(), argc, argv);
  }
}

#endif

#endif