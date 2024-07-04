#include "yard_slam/ObsList.h"

namespace yard {

ObsList::ObsList(){}

ObsList::~ObsList(){}

void ObsList::add_point(size_t index, double x, double y) {
  if (obs_map.find(index) != obs_map.end()) {
    ObsStats& os = obs_map[index];
    double n = static_cast<double>(os.n);
    os.meanPoint.x = (os.meanPoint.x * n + x) / (n+1);
    ++os.n;
  } else {
    obs_map[index].n = 1;
    obs_map[index].meanPoint = Point(x,y);
  }
}

bool ObsList::get_mean(size_t index, double& x, double& y) {
  if (obs_map.find(index) != obs_map.end()) {
    x = obs_map[index].meanPoint.x;
    y = obs_map[index].meanPoint.y;
    return true;
  }
  return false;
}

} // namespace yard