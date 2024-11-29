#include "pico/stdlib.h"
#include <vector>
#include <cmath>

struct Point {
    double x, y;
    Point(double _x = 0.0, double _y = 0.0) : x(_x), y(_y) {}
};

Point bezierPoint(const std::vector<Point>& controlPoints, double t) {
    int n = controlPoints.size() - 1;
    Point result(0, 0);
    for (int i = 0; i <= n; ++i) {
        double binomialCoeff = tgamma(n + 1) / (tgamma(i + 1) * tgamma(n - i + 1));
        double term = pow(1 - t, n - i) * pow(t, i);
        result.x += binomialCoeff * term * controlPoints[i].x;
        result.y += binomialCoeff * term * controlPoints[i].y;
    }
    return result;
}

int main() {
    stdio_init_all();

    std::vector<Point> controlPoints = {
        Point(0, 0),
        Point(1, 2),
        Point(3, 3),
        Point(4, 0)
    };

    int steps = 100;
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Point p = bezierPoint(controlPoints, t);
        printf("Point on curve at t=%.2f: (%.2f, %.2f)\n", t, p.x, p.y);
    }

    return 0;
}