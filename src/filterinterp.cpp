#include "filterinterp.h"

Line2D FilterInterp::processWithOffsets(std::vector<Polar2D*> rawPoints, std::vector<double> angles) {

    std::vector<Point2D> allUsablePoints(rawPoints.size());

    for (auto &angle : angles) {
        // Find the closest point, assuming the array starts at an angle of pi / 2 and goes counter-clockwise
        auto startIndex = static_cast<unsigned long>(round((angle - M_PI_2) / M_2_PI * (double) rawPoints.size()));
        // TODO search around using binary search to find the proper point
        Polar2D* startPoint = rawPoints[startIndex];



        // Work out linearly in both directions checking to see if the change in slope is within the margin,
        // adding points to the usable points as we go
        Point2D lastPoint;
        double slope;
        bool hasRun;

        int rOffset = 1;
        lastPoint = polarToCartesian(startPoint);
        slope = 0;
        hasRun = false;
        do {
            Point2D currentPoint = polarToCartesian(rawPoints.at(startIndex + rOffset));
            double lastSlope = slope;
            slope = (currentPoint.y - lastPoint.y) / (currentPoint.x - lastPoint.x);
            if (abs(slope - lastSlope) > margin) {
                if (hasRun) {
                    break;
                } else {
                    hasRun = true;
                }
            }
            allUsablePoints.push_back(lastPoint);
            lastPoint = currentPoint;
            rOffset++;
        } while (true);

        int lOffset = 1;
        lastPoint = polarToCartesian(startPoint);
        slope = 0;
        hasRun = false;
        do {
            Point2D currentPoint = polarToCartesian(rawPoints.at(startIndex - rOffset));
            double lastSlope = slope;
            slope = (lastPoint.y - currentPoint.y) / (lastPoint.x - currentPoint.x );
            if (abs(slope - lastSlope) > margin) {
                if (hasRun) {
                    break;
                } else {
                    hasRun = true;
                }
            }
            allUsablePoints.push_back(lastPoint);
            lastPoint = currentPoint;
            lOffset++;
        } while (true);

        // Run a linear interpolation
        Eigen::MatrixXf A(allUsablePoints.size(), 2);
        Eigen::VectorXf b(allUsablePoints.size());
        for (unsigned long i = 0; i < allUsablePoints.size(); i++) {
            A(i, 0) = 1;
            A(i, 1) = allUsablePoints.at(i).x;
            b(i)= allUsablePoints.at(i).y;
        }
        Eigen::VectorXf x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        return Line2D {x(0), x(1)};
    }
}