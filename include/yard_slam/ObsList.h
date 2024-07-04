#include <map>

#pragma once

namespace yard {

struct Point {
  Point() :
    x(0.0),
    y(0.0) {}
  Point(double x1, double y1) {
    x = x1;
    y = y1;
  }

  double x;
  double y;
};

struct ObsStats {
  size_t n;
  Point meanPoint;
};

class ObsList {
  public:
    ObsList();
    ~ObsList();

    void add_point(size_t index, double x, double y);
    bool get_mean(size_t index, double& x, double& y);
    void clear() {obs_map.clear();};

    std::map<size_t, ObsStats> obs_map;
};

} // namespace yard