#pragma once

#include <vector>
#include <cmath>
#include "types.hpp"

struct Line {
    double x0, y0, x1, y1;
};

struct World {
    std::vector<Line> lines;
    
    void addLine(double x0, double y0, double x1, double y1) {
        lines.push_back(Line{x0, y0, x1, y1});
    }

    void addRect(double x0, double y0, double x1, double y1) {
        addLine(x0, y0, x1, y0); // ->
        addLine(x1, y0, x1, y1); // ^
        addLine(x1, y1, x0, y1); // <-
        addLine(x0, y1, x0, y0); // V
    }

    double castRay(double originX, double originY, double angle, double maxRange=90) const {
        /*
        P + tR = Q + uS
        where P and Q are the points, R and S are the direction vectors
        ray cast need to solve for t and u

        (P + tR) x S = (Q + uS) x S
        PxS + tRxS = QxS + uSxS
        PxS + tRxS = QxS
        tRxS = QxS - PxS
            = (Q - P) x S
        t = ((Q - P) x S) / (R x S)

        (P + tR) x R = (Q + uS) x R
        PxR + tRxR = QxR + uSxR
        PxR = QxR + uSxR
        PxR - QxR = uSxR
        (P - Q) x R = uSxR
        ((P - Q) x R) / (S x R) = u
        u = ((P - Q) x R) / (S x R)

        i -> j = 1
        j -> i = -1

        R x S = (Rx, Ry) x (Sx, Sy)
        = Rx*Sx + Rx*Sy + RySx + RySy
        = 0 + Rx*Sy + RySx + 0 applied i,j and j,i products
        = Rx*Sy - Ry*Sx

        S x R = -(R x S)

        (Q-P) x S = (Qx - Px, Qy - Py) x (Sx, Sy)
        = (Qx - Px) * Sx + (Qx - Px) * Sy + (Qy - Py) * Sx + (Qy - Py) * Sy
        = 0 + (Qx - Px) * Sy - (Qy - Py) * Sx + 0

        (P-Q) x R = (Px - Qx) * Ry - (Py - Qy) * Rx
        */
        const double Rx = std::cos(angle), Ry = std::sin(angle); // R
        double bestRange = maxRange;

        for (const Line& line : lines) {
            const double Sx = line.x1 - line.x0;
            const double Sy = line.y1 - line.y0;

            const double RxS = Rx * Sy - Ry * Sx; // R x S
            const double SxR = -1 * RxS;
            if (std::abs(RxS) < 1e-12) continue; // parallel line
            
            const double QxmPx = line.x0 - originX;
            const double QymPy = line.y0 - originY;
            const double QmPxS = QxmPx * Sy - QymPy * Sx; // (Q - P) x S
            const double t = QmPxS / RxS; // ((Q - P) x S) / (R x S) 
            
            const double PxmQx = originX - line.x0;
            const double PymQy = originY - line.y0;
            const double PmQxS = PxmQx * Ry - PymQy * Rx;
            const double u = PmQxS / SxR;
            // t >= 0 and u >= 0 means infront of robot
            if (t >= 0.0 && u >= 0.0 && u <= 1.0 && t < bestRange) {
                bestRange = t;
            }
        }

        return bestRange;
    }
};

LaserScan simulateScan(const World& world, LaserType type, const Pose2D& pose, double timestamp, int numBeams = 180, double maxRange = 90.0);