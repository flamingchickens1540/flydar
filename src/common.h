//
// Created by Jonathan Edelman on 2019-01-10.
//

#ifndef LIDAR_COMMON_H
#define LIDAR_COMMON_H

#endif //LIDAR_COMMON_H

struct Polar2D {
    double th, r;
};

struct Point2D {
    double x, y;
};

struct Line2D {
    double m, b;
};

Point2D polarToCartesian(Polar2D *p);