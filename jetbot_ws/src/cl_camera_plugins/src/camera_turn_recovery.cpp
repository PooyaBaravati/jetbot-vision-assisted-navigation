#include <cl_camera_plugins/state.hpp>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/cost_values.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>
#include <boost/thread/locks.hpp>
#include <mutex>
#include <memory>
#include <atomic>
#include <algorithm>
#include <stdexcept>

namespace cl_camera_plugins {
class CameraTurnRecovery : public nav_core::RecoveryBehavior {
  ros::NodeHandle nh_;
  ros::CallbackQueue queue_;
  ros::Subscriber camera_sub_,scan_sub_,map_sub_,updates_sub_,marks_sub_,cancel_sub_,goal_sub_;
  ros::Publisher velocity_,state_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  tf2_ros::Buffer* tf_=nullptr;
  costmap_2d::Costmap2DROS* local_=nullptr;
  std::mutex mutex_;
  FreeCounter free_;
  sensor_msgs::LaserScan::ConstPtr scan_;
  double scan_received_=-1,map_received_=-1,marks_received_=-1,zero_marks_since_=-1;
  uint32_t camera_marks_=0;
  ros::Time map_stamp_;
  std::atomic<bool> interrupted_{false};
  double speed_=0.6,timeout_=1.0,map_timeout_=0.75,max_duration_=30.0,margin_=0.08,handoff_delay_=2.5;
  int free_frames_=5;
  std::string last_state_;
  void state(const std::string& s) {
    if (s==last_state_) return;
    last_state_=s;
    std_msgs::String m; m.data=s; state_.publish(m); ROS_INFO("CL recovery: %s",s.c_str());
  }
  void stop() { velocity_.publish(geometry_msgs::Twist()); }
  void camera(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_); free_.observe(msg->data,monotonicNow(),timeout_);
  }
  void scan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_); scan_=msg; scan_received_=monotonicNow();
  }
  void map(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_); map_received_=monotonicNow(); map_stamp_=msg->header.stamp;
  }
  void updates(const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_); map_received_=monotonicNow(); map_stamp_=msg->header.stamp;
  }
  void marks(const std_msgs::UInt32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now=monotonicNow();
    marks_received_=now;
    if (msg->data==0) {
      if (camera_marks_!=0 || zero_marks_since_<0) zero_marks_since_=now;
    } else {
      zero_marks_since_=-1;
    }
    camera_marks_=msg->data;
  }
  void cancel(const actionlib_msgs::GoalID::ConstPtr&) { interrupted_=true; }
  void goal(const move_base_msgs::MoveBaseActionGoal::ConstPtr&) { interrupted_=true; }

  bool turningClear(const sensor_msgs::LaserScan& scan) {
    // No blocking mutex/TF wait while commanding motion.
    auto* map=local_->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex(),boost::try_to_lock);
    if (!lock.owns_lock()) return false;
    double radius=0;
    for (const auto& p:local_->getRobotFootprint()) radius=std::max(radius,std::hypot(p.x,p.y));
    if (radius<=0) return false;
    radius+=margin_;
    try {
      auto pose=tf_->lookupTransform(local_->getGlobalFrameID(),local_->getBaseFrameID(),ros::Time(0),ros::Duration(0));
      const double age=(ros::Time::now()-pose.header.stamp).toSec();
      if (!pose.header.stamp.isZero() && (age>timeout_ || age < -0.1)) return false;
      const double x=pose.transform.translation.x,y=pose.transform.translation.y;
      const double res=map->getResolution();
      // Cover the entire swept disk, not just its perimeter; reject unknown cells.
      unsigned int lx,ly,hx,hy;
      if (!map->worldToMap(x-radius-res,y-radius-res,lx,ly) ||
          !map->worldToMap(x+radius+res,y+radius+res,hx,hy)) return false;
      for (unsigned int ix=lx;ix<=hx;++ix) for (unsigned int iy=ly;iy<=hy;++iy) {
        double wx,wy; map->mapToWorld(ix,iy,wx,wy);
        if (std::hypot(wx-x,wy-y)<=radius+res*0.7072 && map->getCost(ix,iy)>=costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          return false;
      }
      lock.unlock();
      if (scan.ranges.size()<12 || !std::isfinite(scan.angle_increment) ||
          !std::isfinite(scan.angle_min) || std::abs(scan.angle_increment)*(scan.ranges.size()-1)<6.0 ||
          !std::isfinite(scan.range_max) || scan.range_max<=radius || !std::isfinite(scan.range_min)) return false;
      auto tr=tf_->lookupTransform(local_->getBaseFrameID(),scan.header.frame_id,scan.header.stamp,ros::Duration(0));
      int valid[12]={0},total[12]={0};
      for (size_t i=0;i<scan.ranges.size();++i) {
        int sector=std::min(11,int(i*12/scan.ranges.size())); ++total[sector];
        double r=scan.ranges[i];
        if (std::isinf(r) && r>0) r=scan.range_max;
        if (!std::isfinite(r) || r<scan.range_min || r>scan.range_max) continue;
        ++valid[sector];
        geometry_msgs::PointStamped p,q; p.header=scan.header;
        double a=scan.angle_min+i*scan.angle_increment;
        p.point.x=r*std::cos(a); p.point.y=r*std::sin(a); tf2::doTransform(p,q,tr);
        if (std::hypot(q.point.x,q.point.y)<radius) return false;
      }
      for (int i=0;i<12;++i) if (!total[i] || valid[i]*2<total[i]) return false;
      return true;
    } catch (const tf2::TransformException&) { return false; }
  }
public:
  ~CameraTurnRecovery() override { if (spinner_) spinner_->stop(); }
  void initialize(std::string name,tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS*,costmap_2d::Costmap2DROS* local) override {
    if (local_) return;
    tf_=tf; local_=local; nh_=ros::NodeHandle("~/"+name); nh_.setCallbackQueue(&queue_);
    nh_.param("turn_speed",speed_,0.6); nh_.param("camera_timeout",timeout_,1.0);
    nh_.param("costmap_timeout",map_timeout_,0.75); nh_.param("max_duration",max_duration_,30.0);
    nh_.param("turn_margin",margin_,0.08); nh_.param("free_frames",free_frames_,5);
    nh_.param("free_handoff_delay",handoff_delay_,2.5);
    if (speed_<=0 || speed_>0.6 || timeout_<=0 || map_timeout_<=0 || max_duration_<=0 || margin_<0 ||
        handoff_delay_<0 || handoff_delay_>10.0 || free_frames_<1)
      throw std::runtime_error("Invalid CL recovery parameters");
    velocity_=nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    state_=nh_.advertise<std_msgs::String>("/cl_camera_recovery/state",1,true);
    camera_sub_=nh_.subscribe("/camera/blocked",1,&CameraTurnRecovery::camera,this);
    scan_sub_=nh_.subscribe("/scan",1,&CameraTurnRecovery::scan,this);
    map_sub_=nh_.subscribe("/move_base/local_costmap/costmap",1,&CameraTurnRecovery::map,this);
    updates_sub_=nh_.subscribe("/move_base/local_costmap/costmap_updates",1,&CameraTurnRecovery::updates,this);
    marks_sub_=nh_.subscribe("/move_base/local_costmap/camera_obstacle_layer/active_mark_count",1,&CameraTurnRecovery::marks,this);
    cancel_sub_=nh_.subscribe("/move_base/cancel",1,&CameraTurnRecovery::cancel,this);
    goal_sub_=nh_.subscribe("/move_base/goal",1,&CameraTurnRecovery::goal,this);
    spinner_.reset(new ros::AsyncSpinner(1,&queue_)); spinner_->start(); state("READY");
  }
  void runBehavior() override {
    if (!local_) return;
    interrupted_=false;
    stop(); state("CHECKING");
    const double start=monotonicNow();
    ros::WallRate rate(10);
    double previous=start;
    double free_ready_since=-1;
    while (ros::ok() && monotonicNow()-start<max_duration_) {
      const double now=monotonicNow();
      if (interrupted_) { stop(); state("INTERRUPTED"); return; }
      if (now-previous>0.5) { stop(); state("WAITING_AFTER_CONTROL_DELAY"); previous=now; rate.sleep(); continue; }
      previous=now;
      sensor_msgs::LaserScan::ConstPtr s;
      bool ready,fresh,camera_layer_fresh,post_free_costmap;
      uint32_t camera_marks;
      { std::lock_guard<std::mutex> lock(mutex_);
        ready=free_.ready(now,timeout_,free_frames_);
        fresh=free_.fresh(now,timeout_) && scan_ && now-scan_received_<=timeout_ &&
              map_received_>=0 && now-map_received_<=map_timeout_;
        if (fresh) {
          const double sa=(ros::Time::now()-scan_->header.stamp).toSec();
          const double ma=(ros::Time::now()-map_stamp_).toSec();
          fresh=!scan_->header.stamp.isZero() && !map_stamp_.isZero() && sa>=-0.1 && sa<=timeout_ && ma>=-0.1 && ma<=map_timeout_;
        }
        camera_layer_fresh=marks_received_>=0 && now-marks_received_<=map_timeout_;
        post_free_costmap=free_ready_since>=0 && map_received_>free_ready_since;
        camera_marks=camera_marks_;
        s=scan_;
      }
      if (!fresh) { free_ready_since=-1; stop(); state("WAITING_FOR_FRESH_DATA"); rate.sleep(); continue; }
      if (ready) {
        stop();
        if (free_ready_since<0) free_ready_since=now;
        if (camera_layer_fresh && post_free_costmap) {
          if (camera_marks>0 && now-free_ready_since<handoff_delay_) {
            state("WAITING_FOR_GLOBAL_PLANNER_HANDOFF"); rate.sleep(); continue;
          }
          state(camera_marks>0 ? "FREE_OBSTACLE_RETAINED_RETURN_TO_PLANNER" :
                                 "FREE_CLEAR_RETURN_TO_PLANNER");
          return;
        }
        state("WAITING_FOR_POST_FREE_COSTMAP_UPDATE"); rate.sleep(); continue;
      }
      free_ready_since=-1;
      if (!turningClear(*s)) { stop(); state("WAITING_FOR_TURN_CLEARANCE_OR_MAP"); rate.sleep(); continue; }
      // Check again after clearance computation in case a cancellation arrived.
      if (interrupted_) { stop(); state("INTERRUPTED"); return; }
      if (monotonicNow()-now>0.1) { stop(); state("WAITING_AFTER_CLEARANCE_CHECK_DELAY"); rate.sleep(); continue; }
      geometry_msgs::Twist cmd; cmd.angular.z=speed_; velocity_.publish(cmd);
      state("TURNING_FOR_FREE_PATH");
      rate.sleep();
    }
    stop(); state("STOP_TIMEOUT");
  }
};
}
PLUGINLIB_EXPORT_CLASS(cl_camera_plugins::CameraTurnRecovery,nav_core::RecoveryBehavior)
