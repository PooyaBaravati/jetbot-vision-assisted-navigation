#include <cl_camera_plugins/state.hpp>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/cost_values.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mutex>
#include <memory>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tf2/utils.h>
// CL_FREE_SECTOR_V34

namespace cl_camera_plugins {
class ExpiringCameraLayer : public costmap_2d::Layer {
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  ros::Subscriber scan_sub_, camera_sub_;
  ros::Publisher mark_count_pub_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::mutex mutex_;
  ExpiringMarks marks_;
  std::vector<Mark> snapshot_;
  std::vector<std::pair<double,double> > transition_anchor_;
  double ttl_=2.0, timeout_=1.0, scan_timeout_=0.75, last_camera_=-1, last_scan_=-1;
  double resolution_=0.05, mark_radius_=0.0;
  bool transition_only_=false, obstacle_scan_active_=false, preserve_on_reset_=false;
  std::string frame_;
  bool camera_blocked_=true;
  double free_clear_hold_=0.8, free_clear_range_=0.45;

  void camera(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now=monotonicNow();
    if (msg->data || last_camera_<0 || now-last_camera_>timeout_) marks_.resetFreeEvidence();
    camera_blocked_=msg->data;
    last_camera_=now;
  }
  void scan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    const double now=monotonicNow();
    { std::lock_guard<std::mutex> lock(mutex_);
      if (last_camera_<0 || now-last_camera_>timeout_) return;
    }
    const double age=(ros::Time::now()-msg->header.stamp).toSec();
    if (msg->header.stamp.isZero() || age < -0.1 || age>scan_timeout_ ||
        !std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_increment) ||
        !std::isfinite(msg->range_min) || !std::isfinite(msg->range_max) ||
        msg->range_max<=msg->range_min || msg->ranges.size()>1000) return;
    // A FREE scan has only +inf ranges and needs no transform.
    bool has_finite_obstacle=false;
    for (const float r:msg->ranges) {
      if (std::isfinite(r) && r>=msg->range_min && r<=msg->range_max) {
        has_finite_obstacle=true;
        break;
      }
    }
    if (!has_finite_obstacle) {
      // Only a complete +infinity scan is evidence of FREE. NaNs, empty
      // scans and invalid ranges must never erase obstacles.
      bool all_free=!msg->ranges.empty();
      for (const float r:msg->ranges)
        if (!(std::isinf(r) && r>0)) all_free=false;
      if (!all_free) {
        std::lock_guard<std::mutex> lock(mutex_);
        marks_.resetFreeEvidence();
        return;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_scan_=now;
        obstacle_scan_active_=false;
        transition_anchor_.clear();
        if (camera_blocked_ || now-last_camera_>timeout_) {
          marks_.resetFreeEvidence(); return;
        }
      }
      try {
        auto transform=tf_->lookupTransform(frame_,msg->header.frame_id,
                                            msg->header.stamp,ros::Duration(0.15));
        const double yaw=tf2::getYaw(transform.transform.rotation);
        const double end_angle=msg->angle_min+
                              (msg->ranges.size()-1)*msg->angle_increment;
        const double amin=std::max(-0.35, std::min(double(msg->angle_min),end_angle));
        const double amax=std::min( 0.35, std::max(double(msg->angle_min),end_angle));
        std::lock_guard<std::mutex> lock(mutex_);
        const double checked_now=monotonicNow();
        if (camera_blocked_ || checked_now-last_camera_>timeout_ ||
            checked_now-now>scan_timeout_) {
          marks_.resetFreeEvidence(); return;
        }
        const unsigned int removed=marks_.clearFreeSector(
            transform.transform.translation.x,transform.transform.translation.y,yaw,
            amin,amax,msg->range_min,std::min(double(msg->range_max),free_clear_range_),
            checked_now,free_clear_hold_,std::min(scan_timeout_,0.25),
            mark_radius_+0.707107*resolution_);
        if (removed) ROS_INFO("CL FREE clearing [%s]: removed %u camera marks",name_.c_str(),removed);
      } catch (const tf2::TransformException& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        marks_.resetFreeEvidence();
        ROS_WARN_THROTTLE(2.0,"CL FREE clearing skipped: %s",e.what());
      }
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      marks_.resetFreeEvidence();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (transition_only_ && obstacle_scan_active_) {
        // Refresh the first blocked-direction marks without painting a ring.
        for (const auto& p:transition_anchor_)
          marks_.add(p.first,p.second,now,resolution_);
        last_scan_=now;
        return;
      }
    }
    try {
      // Briefly wait for TF to catch up with the scan timestamp.
      auto transform=tf_->lookupTransform(frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(0.15));
      std::vector<Mark> new_marks;
      for (size_t i=0;i<msg->ranges.size();++i) {
        const double r=msg->ranges[i];
        if (!std::isfinite(r) || r<msg->range_min || r>msg->range_max) continue;
        double angle=msg->angle_min+i*msg->angle_increment;
        geometry_msgs::PointStamped p,q;
        p.header=msg->header; p.point.x=r*std::cos(angle); p.point.y=r*std::sin(angle);
        tf2::doTransform(p,q,transform);
        new_marks.push_back({q.point.x,q.point.y,now});
      }
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& p:new_marks) marks_.add(p.x,p.y,now,resolution_);
      if (transition_only_) {
        transition_anchor_.clear();
        for (const auto& p:new_marks)
          transition_anchor_.push_back(std::make_pair(p.x,p.y));
        obstacle_scan_active_=true;
      }
      last_scan_=now;
    } catch (const tf2::TransformException& e) {
      ROS_WARN_THROTTLE(2.0,"CL camera transform unavailable: %s",e.what());
    }
  }
public:
  ~ExpiringCameraLayer() override { if (spinner_) spinner_->stop(); }
  void onInitialize() override {
    nh_=ros::NodeHandle("~/"+name_); nh_.setCallbackQueue(&queue_);
    nh_.param("obstacle_ttl",ttl_,2.0); nh_.param("camera_timeout",timeout_,1.0);
    nh_.param("scan_timeout",scan_timeout_,0.75);
    nh_.param("free_clear_hold",free_clear_hold_,0.8);
    nh_.param("free_clear_range",free_clear_range_,0.45);
    if (!std::isfinite(free_clear_hold_) || free_clear_hold_<0.8 ||
        !std::isfinite(free_clear_range_) || free_clear_range_<=0 || free_clear_range_>0.45)
      throw std::runtime_error("Invalid CL FREE clearing parameters");
    nh_.setParam("free_clear_hold",free_clear_hold_);
    nh_.setParam("free_clear_range",free_clear_range_);
    nh_.param("transition_only",transition_only_,false);
    nh_.param("mark_radius",mark_radius_,0.0);
    nh_.param("preserve_on_reset",preserve_on_reset_,false);
    if (ttl_<=0 || timeout_<=0 || scan_timeout_<=0 || mark_radius_<0 || mark_radius_>1.0)
      throw std::runtime_error("Invalid CL camera-layer parameters");
    frame_=layered_costmap_->getGlobalFrameID();
    resolution_=layered_costmap_->getCostmap()->getResolution();
    enabled_=true; current_=false;
    scan_sub_=nh_.subscribe("/camera_virtual_scan",1,&ExpiringCameraLayer::scan,this);
    camera_sub_=nh_.subscribe("/camera/blocked",1,&ExpiringCameraLayer::camera,this);
    mark_count_pub_=nh_.advertise<std_msgs::UInt32>("active_mark_count",1,true);
    spinner_.reset(new ros::AsyncSpinner(1,&queue_)); spinner_->start();
    ROS_INFO("CL expiring camera layer: TTL %.2fs; transition_only=%s; mark_radius=%.2fm; preserve_on_reset=%s",
             ttl_,transition_only_ ? "true" : "false",mark_radius_,
             preserve_on_reset_ ? "true" : "false");
  }
  void updateBounds(double,double,double,double* min_x,double* min_y,double* max_x,double* max_y) override {
    // Dirty current and previous cells only. Previous cells erase expired
    // marks; the following inflation layer expands these bounds as required.
    const std::vector<Mark> previous=snapshot_;
    std_msgs::UInt32 count;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now=monotonicNow();
      snapshot_=marks_.snapshot(now,ttl_);
      current_=last_camera_>=0 && now-last_camera_<=timeout_ && last_scan_>=0 && now-last_scan_<=scan_timeout_;
      count.data=static_cast<uint32_t>(snapshot_.size());
    }
    const double pad=resolution_+mark_radius_;
    for (const auto& p:previous) {
      *min_x=std::min(*min_x,p.x-pad); *min_y=std::min(*min_y,p.y-pad);
      *max_x=std::max(*max_x,p.x+pad); *max_y=std::max(*max_y,p.y+pad);
    }
    for (const auto& p:snapshot_) {
      *min_x=std::min(*min_x,p.x-pad); *min_y=std::min(*min_y,p.y-pad);
      *max_x=std::max(*max_x,p.x+pad); *max_y=std::max(*max_y,p.y+pad);
    }
    mark_count_pub_.publish(count);
  }
  void updateCosts(costmap_2d::Costmap2D& master,int min_i,int min_j,int max_i,int max_j) override {
    const double resolution=master.getResolution();
    const int radius_cells=static_cast<int>(std::ceil(mark_radius_/resolution));
    for (const auto& p:snapshot_) {
      unsigned int x,y;
      if (!master.worldToMap(p.x,p.y,x,y)) continue;
      for (int dy=-radius_cells;dy<=radius_cells;++dy) {
        for (int dx=-radius_cells;dx<=radius_cells;++dx) {
          if (std::hypot(dx*resolution,dy*resolution)>mark_radius_+0.5*resolution) continue;
          const int mx=static_cast<int>(x)+dx;
          const int my=static_cast<int>(y)+dy;
          if (mx<min_i || my<min_j || mx>=max_i || my>=max_j || mx<0 || my<0 ||
              mx>=static_cast<int>(master.getSizeInCellsX()) || my>=static_cast<int>(master.getSizeInCellsY())) continue;
          master.setCost(static_cast<unsigned int>(mx),static_cast<unsigned int>(my),costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    marks_.resetFreeEvidence();
    if (preserve_on_reset_) {
      // Recovery may clear the master grid, but updateBounds/updateCosts will
      // restore these still-live marks on the next costmap cycle. Their
      // timestamps are untouched, so normal TTL expiration still applies.
      current_=false;
      return;
    }
    marks_.clear(); transition_anchor_.clear(); obstacle_scan_active_=false;
    last_scan_=-1; current_=false;
  }
};
}
PLUGINLIB_EXPORT_CLASS(cl_camera_plugins::ExpiringCameraLayer,costmap_2d::Layer)
