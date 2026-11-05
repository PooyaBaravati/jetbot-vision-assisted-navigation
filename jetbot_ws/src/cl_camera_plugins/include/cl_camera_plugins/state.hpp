#pragma once
#include <map>
#include <vector>
#include <utility>
#include <cmath>
#include <chrono>
namespace cl_camera_plugins {
inline double monotonicNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Mark { double x, y, seen; };
class ExpiringMarks {
  std::map<std::pair<int,int>, Mark> marks_;

  // CL_FREE_SECTOR_V34: each mark must remain in a fresh FREE sector.
  std::map<std::pair<int,int>, std::pair<double,double> > free_windows_;
public:
  void resetFreeEvidence() { free_windows_.clear(); }
  unsigned int clearFreeSector(double ox, double oy, double yaw,
      double amin, double amax, double rmin, double rmax,
      double now, double hold, double max_gap, double cell_pad) {
    unsigned int removed=0;
    for (auto it=marks_.begin(); it!=marks_.end();) {
      const double dx=it->second.x-ox, dy=it->second.y-oy;
      const double distance=std::hypot(dx,dy);
      const double angle=std::atan2(std::sin(std::atan2(dy,dx)-yaw),
                                    std::cos(std::atan2(dy,dx)-yaw));
      const double margin=distance>cell_pad ? std::asin(cell_pad/distance) : 3.141593;
      const bool inside=distance>=rmin+cell_pad && distance<=rmax-cell_pad &&
                        angle>=amin+margin && angle<=amax-margin;
      if (!inside) { free_windows_.erase(it->first); ++it; continue; }
      auto evidence=free_windows_.find(it->first);
      if (evidence==free_windows_.end() || now-evidence->second.second>max_gap ||
          now<evidence->second.second) {
        free_windows_[it->first]={now,now}; ++it; continue;
      }
      evidence->second.second=now;
      if (now-evidence->second.first>=hold) {
        free_windows_.erase(it->first); it=marks_.erase(it); ++removed;
      } else ++it;
    }
    return removed;
  }
  void add(double x, double y, double now, double resolution) {
    marks_[{int(std::floor(x/resolution)),int(std::floor(y/resolution))}]={x,y,now};
  }
  std::vector<Mark> snapshot(double now, double ttl) {
    std::vector<Mark> out;
    for (auto it=marks_.begin(); it!=marks_.end();) {
      if (now-it->second.seen >= ttl) { free_windows_.erase(it->first); it=marks_.erase(it); }
      else { out.push_back(it->second); ++it; }
    }
    return out;
  }
  void clear() { marks_.clear(); free_windows_.clear(); }
};
class FreeCounter {
  int count_=0;
  double seen_=-1;
public:
  void observe(bool blocked, double now, double max_gap) {
    if (seen_<0 || now-seen_>max_gap) count_=0;
    count_=blocked ? 0 : (count_<10000 ? count_+1 : count_);
    seen_=now;
  }
  bool fresh(double now, double timeout) const { return seen_>=0 && now-seen_<=timeout; }
  bool ready(double now, double timeout, int required) const { return fresh(now,timeout) && count_>=required; }
};
}
