//
// Created by Jonathan Edelman on 2019-01-10.
//

#include <cmath>
#include "common.h"

Point2D polarToCartesian(Polar2D *p) {
    return Point2D{
        p->r * cos(p->th),
        p->r * sin(p->th)
    };
}