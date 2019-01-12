//
// Created by wangl on 1/11/19.
//

#ifndef LIDAR_FILTERINTERP_H
#define LIDAR_FILTERINTERP_H

//
// Created by Jonathan Edelman on 2019-01-10.
//

#include <math.h>
#include "common.h"
#include <vector>
#include <Eigen/Dense>

class FilterInterp {

private:
    double margin = 0.05;

public:
    Line2D processWithOffsets(std::vector<Polar2D*> rawPoints, std::vector<double> angles);
};

#endif //LIDAR_FILTERINTERP_H
