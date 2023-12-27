#ifndef __AVERAGING_H__
#define __AVERAGING_H__

#include <vector>

#include "manif/manif.h"

manif::SE3d mean(const std::vector<manif::SE3d>& transforms);
manif::SE3d inlier_mean(const std::vector<manif::SE3d>& transforms);

#endif  // __AVERAGING_H__
