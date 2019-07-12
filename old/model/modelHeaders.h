// -*- mode: c++; -*-

#ifndef CREEK_MODEL_HEADERS_H
#define CREEK_MODEL_HEADERS_H

//
// choreonoid
//
#ifdef USE_CNOID_MODEL
#include "../model/cnoidBody.hpp"
#include <cnoid/EigenUtil>
namespace model = cnoid;

//
// openhrp
//
#elif defined USE_HRP_MODEL
#include "../model/hrpBody.hpp"
#include <hrpUtil/MatrixSolvers.h>
namespace model = hrp;
#endif

#endif
