#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
//#include <tf_conversions/tf_eigen.h>
#include <ros/callback_queue.h>

#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/UploadTrajectory.h"
#include "crazyflie_driver/TrajectoryRef.h"
#undef major
#undef minor
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/GoTo.h"
#include "crazyflie_driver/StartTrajectory.h"
#include "crazyflie_driver/SetGroupMask.h"
#include "crazyflie_driver/getPosSetPoint.h"

#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float32.h"

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud.h>
#include "crazyflie_driver/state_tg.h"

//#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "control_modified.h"
#include "commander.h"
#include <crazyflie_cpp/Crazyflie.h>

/*formation control dependencies*/
#include <string>
#include <eiquadprog.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include "crazyflie_driver/IdPos.h"
#include <stdint.h>

// debug test
#include <signal.h>
#include <csignal> // or C++ style alternative

// Motion Capture
#ifdef ENABLE_VICON
#include "libmotioncapture/vicon.h"
#endif
#ifdef ENABLE_OPTITRACK
#include "libmotioncapture/optitrack.h"
#endif
#ifdef ENABLE_PHASESPACE
#include "libmotioncapture/phasespace.h"
#endif

// Object tracker
#include <libobjecttracker/object_tracker.h>
#include <libobjecttracker/cloudlog.hpp>

#include <fstream>
#include <future>
#include <mutex>
#include <wordexp.h> // tilde expansion

#include <stdio.h>

#define DYN_ERR 0.02
/*
Threading
 * There are 2N+1 threads, where N is the number of groups (== number of unique channels)
 * The main thread uses the VICON SDK to query vicon; Once a new frame comes in, the
   workers (CrazyflieGroup) are notified using a condition variable. Each CrazyflieGroup
   does the objectTracking for its own group and broadcasts the resulting vicon data.
 * One helper thread is used in the server to take care of incoming global service requests.
   Those are forwarded to the groups (using a function call, i.e. the broadcasts run in this thread).
 * Each group has two threads:
   * VICON worker. Waits for new vicon data (using a condition variable) and does the object tracking
     and broadcasts the result.
   * Service worker: Listens to CF-based service calls (such as upload trajectory) and executes
     them. Those can be potentially long, without interfering with the VICON update.
*/

constexpr double pi() { return std::atan(1)*4; }
int isStartTr = 0;
long cnt = 0;

float g_dt=0.01;



Eigen::Vector3f gPositionRef;

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

void logWarn(const std::string& msg)
{
  ROS_WARN("%s", msg.c_str());
}

bool isStopSendTr = false;

class ROSLogger : public Logger
{
public:
  ROSLogger()
    : Logger()
  {
  }

  virtual ~ROSLogger() {}

  virtual void info(const std::string& msg)
  {
    ROS_INFO("%s", msg.c_str());
  }

  virtual void warning(const std::string& msg)
  {
    ROS_WARN("%s", msg.c_str());
  }

  virtual void error(const std::string& msg)
  {
    ROS_ERROR("%s", msg.c_str());
  }
};

static ROSLogger rosLogger;

// TODO this is incredibly dumb, fix it
/*
std::mutex viconClientMutex;

static bool viconObjectAllMarkersVisible(
  ViconDataStreamSDK::CPP::Client &client, std::string const &objName)
{
  std::lock_guard<std::mutex> guard(viconClientMutex);
  using namespace ViconDataStreamSDK::CPP;
  auto output = client.GetMarkerCount(objName);
  if (output.Result != Result::Success) {
    return false;
  }
  bool ok = true;
  for (unsigned i = 0; i < output.MarkerCount; ++i) {
    auto marker = client.GetMarkerName(objName, i);
    if (marker.Result != Result::Success) {
      ROS_INFO("GetMarkerName fail on marker %d", i);
      return false;
    }
    auto position = client.GetMarkerGlobalTranslation(objName, marker.MarkerName);
    if (position.Result != Result::Success) {
      ROS_INFO("GetMarkerGlobalTranslation fail on marker %s",
        std::string(marker.MarkerName).c_str());
      return false;
    }
    if (position.Occluded) {
      ROS_INFO("Interactive object marker %s occluded with z = %f",
        std::string(marker.MarkerName).c_str(), position.Translation[2]);
      ok = false;
      // don't early return; we want to print messages for all occluded markers
    }
  }
  return ok;
}
*/

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    const std::string& frame,
    const std::string& worldFrame,
    bool enable_parameters,
    bool enable_logging,
    int id,
    const std::string& type,
    const std::vector<crazyflie_driver::LogBlock>& log_blocks,
    ros::CallbackQueue& queue,
    bool force_no_cache)
    : m_cf(link_uri, rosLogger)
    , m_tf_prefix(tf_prefix)
    , m_frame(frame)
    , m_worldFrame(worldFrame)
    , m_enableParameters(enable_parameters)
    , m_enableLogging(enable_logging)
    , m_id(id)
    , m_type(type)
    , m_serviceUpdateParams()
    , m_serviceUploadTrajectory()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_serviceGoTo()
    , m_serviceSetGroupMask()
    , m_logBlocks(log_blocks)
    , m_forceNoCache(force_no_cache)
    , m_initializedPosition(false)
    , isNotSendPIng(false)
    , m_controller(id)
//    , m_isHoverOK(false)
  {
    printf("-----------------hello swarmServer --------------------\n");
    ros::NodeHandle n;
    n.setCallbackQueue(&queue);
    m_serviceUploadTrajectory = n.advertiseService(tf_prefix + "/upload_trajectory", &CrazyflieROS::uploadTrajectory, this);
    m_serviceTakeoff = n.advertiseService(tf_prefix + "/takeoff", &CrazyflieROS::takeoff, this);
    m_serviceLand = n.advertiseService(tf_prefix + "/land", &CrazyflieROS::land, this);
    m_serviceGoTo = n.advertiseService(tf_prefix + "/go_to", &CrazyflieROS::goTo, this);
    m_serviceSetGroupMask = n.advertiseService(tf_prefix + "/set_group_mask", &CrazyflieROS::setGroupMask, this);
//    m_traj_ref_sub = n.subscribe(tf_prefix + "/set_state", 100, &CrazyflieROS::aflie_state_traj_cb,this);
    //m_serviceTrajectoryRef=n.advertiseService(tf_prefix + "/set_trajectory_ref", &CrazyflieROS::setTrajectoryRef, this);
    m_PosSetPoint.setZero();
    m_VelSetPoint.setZero();
    m_AccSetPoint.setZero();
    m_currentPosition.setZero();
    m_lastPosition.setZero();
    m_initialPosition.setZero();
    m_positionRef.setZero();
    m_velocityRef.setZero();
    m_accelerationRef.setZero();
    s_tg_tmp.p_x = 0.0;
    s_tg_tmp.p_y = 0.0;
    s_tg_tmp.p_z = -1.0;
    s_tg_tmp.v_x = 0.0;
    s_tg_tmp.v_y = 0.0;
    s_tg_tmp.v_z = 0.0;
    s_tg_tmp.a_x = 0.0;
    s_tg_tmp.a_y = 0.0;
    s_tg_tmp.a_z = 0.0;
    m_PosSetPoint(2)=-1;


    Euler.setZero();
    m_positionRef(2)=-1;
    m_dt=0.01;
    m_commander.initSps(m_PosSetPoint,m_VelSetPoint,m_AccSetPoint);
    m_rpy(0) = -3;
    if (m_enableLogging) {
      m_logFile.open("logcf" + std::to_string(id) + ".csv");
      m_logFile << "time,";
      for (auto& logBlock : m_logBlocks) {
        m_pubLogDataGeneric.push_back(n.advertise<crazyflie_driver::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
        for (const auto& variableName : logBlock.variables) {
          m_logFile << variableName << ",";
        }
      }
      m_logFile << std::endl;
    }

    // m_subscribeJoy = n.subscribe("/joy", 1, &CrazyflieROS::joyChanged, this);
  }

  ~CrazyflieROS()
  {
    m_logBlocks.clear();
    m_logBlocksGeneric.clear();
    m_cf.trySysOff();
    m_logFile.close();
  }

  const std::string& frame() const {
    return m_frame;
  }

  const int id() const {
    return m_id;
  }

  const std::string& type() const {
    return m_type;
  }

  void sendPing() {
    if(!isNotSendPIng){
      m_cf.sendPing();
      }
  }

public:
  bool isNotSendPIng;
  Eigen::Vector3f m_rpy;
  template<class T, class U>
  void updateParam(uint8_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.addSetParam<T>(id, (T)value);
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("[%s] Update parameters", m_frame.c_str());
    m_cf.startSetParamRequest();
    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    std::cout<<"CF "<<m_id<<" Send paramters."<<std::endl;
    m_cf.setRequestedParams();
    return true;
  }


  bool uploadTrajectory(
    crazyflie_driver::UploadTrajectory::Request& req,
    crazyflie_driver::UploadTrajectory::Response& res)
  {
    ROS_INFO("[%s] Upload trajectory", m_frame.c_str());
    isNotSendPIng = true;
    isStopSendTr = true;
    std::vector<Crazyflie::poly4d> pieces(req.pieces.size());
    for (size_t i = 0; i < pieces.size(); ++i) {
      if (   req.pieces[i].poly_x.size() != 8
          || req.pieces[i].poly_y.size() != 8
          || req.pieces[i].poly_z.size() != 8
          || req.pieces[i].poly_yaw.size() != 8) {
        ROS_FATAL("Wrong number of pieces!");
        return false;
      }
      pieces[i].duration = req.pieces[i].duration.toSec();
      for (size_t j = 0; j < 8; ++j) {
        pieces[i].p[0][j] = req.pieces[i].poly_x[j];
        pieces[i].p[1][j] = req.pieces[i].poly_y[j];
        pieces[i].p[2][j] = req.pieces[i].poly_z[j];
        pieces[i].p[3][j] = req.pieces[i].poly_yaw[j];
      }
    }
      m_cf.uploadTrajectory(req.trajectoryId, req.pieceOffset, pieces);

    ROS_INFO("[%s] Uploaded trajectory", m_frame.c_str());

    isNotSendPIng = false;
    isStopSendTr = false;
    return true;
  }

  bool takeoff(
    crazyflie_driver::Takeoff::Request& req,
    crazyflie_driver::Takeoff::Response& res)
  {
    ROS_INFO("[%s] Takeoff", m_frame.c_str());
    
    m_cf.takeoff(req.height, req.duration.toSec(), req.groupMask);

    return true;
  }

  bool land(
    crazyflie_driver::Land::Request& req,
    crazyflie_driver::Land::Response& res)
  {
    ROS_INFO("[%s] Land", m_frame.c_str());

    m_cf.land(req.height, req.duration.toSec(), req.groupMask);

    return true;
  }

  bool goTo(
    crazyflie_driver::GoTo::Request& req,
    crazyflie_driver::GoTo::Response& res)
  {
    ROS_INFO("[%s] GoTo", m_frame.c_str());

    m_cf.goTo(req.goal.x, req.goal.y, req.goal.z, req.yaw, req.duration.toSec(), req.relative, req.groupMask);

    return true;
  }

  bool setGroupMask(
    crazyflie_driver::SetGroupMask::Request& req,
    crazyflie_driver::SetGroupMask::Response& res)
  {
    ROS_INFO("[%s] Set Group Mask", m_frame.c_str());

    m_cf.setGroupMask(req.groupMask);

    return true;
  }

  bool setTrajectoryRef(
            crazyflie_driver::TrajectoryRef::Request& req,
            crazyflie_driver::TrajectoryRef::Response& res)
  {
    //ROS_INFO("[%s] Set Group Mask", m_frame.c_str());

    m_positionRef(0)=req.x;
    m_positionRef(1)=req.y;
    m_positionRef(2)=req.z;
    m_velocityRef(0)=req.vx;
    m_velocityRef(1)=req.vy;
    m_velocityRef(2)=req.vz;
    m_accelerationRef(0)=req.ax;
    m_accelerationRef(1)=req.ay;
    m_accelerationRef(2)=req.az;

    return true;
  }


  void setTakeOffPos(const float& x,const float& y,const float& z){
    m_commander.setTakeOffPos(x, y, z);
    m_initialPosition(0)=x;
    m_initialPosition(1)=y;
    m_initialPosition(2)=z;
    m_lastPosition(0)=x;
    m_lastPosition(1)=y;
    m_lastPosition(2)=z;
    m_controller.l_posVicon(0)=x;
    m_controller.l_posVicon(1)=y;
    m_controller.l_posVicon(2)=z;

    std::cout<<"Initial position:"<<x<<y<<z<<std::endl;
  }

  void aflie_state_traj_cb(const crazyflie_driver::state_tg::ConstPtr& s_tg){
    s_tg_tmp = *s_tg;
    m_PosSetPoint(0) = s_tg_tmp.p_x + m_initialPosition(0);
    m_PosSetPoint(1) = s_tg_tmp.p_y + m_initialPosition(1);
    m_PosSetPoint(2) = s_tg_tmp.p_z;
    //m_PosSetPoint(3) = 0;
    m_VelSetPoint(0) = s_tg_tmp.v_x;
    m_VelSetPoint(1) = s_tg_tmp.v_y;
    m_VelSetPoint(2) = s_tg_tmp.v_z;
    m_AccSetPoint(0) = s_tg_tmp.a_x;
    m_AccSetPoint(1) = s_tg_tmp.a_y;
    m_AccSetPoint(2) = s_tg_tmp.a_z;
    //std::cout<<"flie"<<m_id<<"set point"<<m_PosSetPoint(0)<<" "<<m_PosSetPoint(1)<<" "<<m_PosSetPoint(2)<<std::endl;

  };


  void run(
    ros::CallbackQueue& queue)
  {
    // m_cf.reboot();
    // m_cf.syson();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    auto start = std::chrono::system_clock::now();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);

    m_cf.logReset();

    if (m_enableParameters)
    {
      ROS_INFO("[%s] Requesting parameters...", m_frame.c_str());
      m_cf.requestParamToc(m_forceNoCache);
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
      }
      ros::NodeHandle n;
      n.setCallbackQueue(&queue);
      m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
    }
    auto end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds1 = end1-start;
    ROS_INFO("[%s] reqParamTOC: %f s", m_frame.c_str(), elapsedSeconds1.count());

    // Logging
    if (m_enableLogging) {
      ROS_INFO("[%s] Requesting logging variables...", m_frame.c_str());
      m_cf.requestLogToc(m_forceNoCache);
      auto end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds2 = end2-end1;
      ROS_INFO("[%s] reqLogTOC: %f s", m_frame.c_str(), elapsedSeconds2.count());

      m_logBlocksGeneric.resize(m_logBlocks.size());
      // custom log blocks
      size_t i = 0;
      for (auto& logBlock : m_logBlocks)
      {
        std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
          std::bind(
            &CrazyflieROS::onLogCustom,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

        m_logBlocksGeneric[i].reset(new LogBlockGeneric(
          &m_cf,
          logBlock.variables,
          (void*)&m_pubLogDataGeneric[i],
          cb));
        m_logBlocksGeneric[i]->start(logBlock.frequency / 10);
        ++i;
      }
      auto end3 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds3 = end3-end2;
      ROS_INFO("[%s] logBlocks: %f s", m_frame.c_str(), elapsedSeconds1.count());
    }

    ROS_INFO("Requesting memories...");
    m_cf.requestMemoryToc();

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("[%s] Ready. Elapsed: %f s", m_frame.c_str(), elapsedSeconds.count());
  }

  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        ROS_WARN("[%s] Link Quality low (%f)", m_frame.c_str(), linkQuality);
      }
  }

  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

    ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

    crazyflie_driver::GenericLogData msg;
    msg.header.stamp = ros::Time(time_in_ms/1000.0);
    msg.values = *values;

    m_logFile << time_in_ms / 1000.0 << ",";
    for (const auto& value : *values) {
      m_logFile << value << ",";
    }
    m_logFile << std::endl;

    pub->publish(msg);
  }

  const Crazyflie::ParamTocEntry* getParamTocEntry(
    const std::string& group,
    const std::string& name) const
  {
    return m_cf.getParamTocEntry(group, name);
  }

  void initializePositionIfNeeded(float x, float y, float z)
  {
    if (m_initializedPosition) {
      return;
    }

    m_cf.startSetParamRequest();
    auto entry = m_cf.getParamTocEntry("kalman", "initialX");
    m_cf.addSetParam(entry->id, x);
    entry = m_cf.getParamTocEntry("kalman", "initialY");
    m_cf.addSetParam(entry->id, y);
    entry = m_cf.getParamTocEntry("kalman", "initialZ");
    m_cf.addSetParam(entry->id, z);
    m_cf.setRequestedParams();

    entry = m_cf.getParamTocEntry("kalman", "resetEstimation");
    m_cf.setParam<uint8_t>(entry->id, 1);

    m_initializedPosition = true;
  }
  //added by yhz
  void initSetPoint()
  {
    m_commander.init_sp(&m_PosSetPoint,&m_VelSetPoint,&m_AccSetPoint);
  }

  void getSetpoint()
  {
//        m_commander.command_takeoff(0.02f);
      m_PosSetPoint(0) =  m_initialPosition(0)+gPositionRef(0);//m_positionRef(0);
      m_PosSetPoint(1) =  m_initialPosition(1)+gPositionRef(1);//m_positionRef(1);
      m_PosSetPoint(2) = 0.0f + gPositionRef(2);//m_positionRef(2);
      m_PosSetPoint(3) = 0.0f; //yaw
      //std::cout<<"Pos ref:"<<m_PosSetPoint(0)<<","<<m_PosSetPoint(1)<<","<<m_PosSetPoint(2)<<std::endl;
//
      m_VelSetPoint(0) = 0.0f+m_velocityRef(0);
      m_VelSetPoint(1) = 0.0f+m_velocityRef(1);
      m_VelSetPoint(2) = 0.0f+m_velocityRef(2);

      m_AccSetPoint(0) = 0.0f+m_accelerationRef(0);
      m_AccSetPoint(1) = 0.0f+m_accelerationRef(1);
      m_AccSetPoint(2) = 0.0f+m_accelerationRef(2);

  }


  void getPositionSetPoint()
    {
      ros::spinOnce(); //added by xs
      /*for(auto& cf : m_cfs)
      {
        //cf->getSetpoint();
        cf->getSetpoint_from_topic();
      }*/
    }

  void giveCurrentPos(float x, float y, float z)
  {
      m_currentPosition(0) = x;
      m_currentPosition(1) = y;
      m_currentPosition(2) = z;
  }

  void giveCurrentPos(pcl::PointCloud<pcl::PointXYZ>::Ptr pmarkers)
  {
    Eigen::Vector3f temp;
    temp=m_lastPosition;
    for(int i=0;i<pmarkers->size();i++)
     {

     }
  }


private:
  Crazyflie m_cf;
  std::string m_tf_prefix;
  std::string m_frame;
  std::string m_worldFrame;
  bool m_enableParameters;
  bool m_enableLogging;
  int m_id;
  std::string m_type;

  ros::ServiceServer m_serviceUpdateParams;
  ros::ServiceServer m_serviceUploadTrajectory;
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  ros::ServiceServer m_serviceGoTo;
  ros::ServiceServer m_serviceSetGroupMask;
  ros::ServiceServer m_serviceTrajectoryRef;
  crazyflie_driver::state_tg s_tg_tmp;

  ros::Subscriber m_traj_ref_sub;
  std::vector<crazyflie_driver::LogBlock> m_logBlocks;
  std::vector<ros::Publisher> m_pubLogDataGeneric;
  std::vector<std::unique_ptr<LogBlockGeneric> > m_logBlocksGeneric;

  ros::Subscriber m_subscribeJoy;

  std::ofstream m_logFile;
  bool m_forceNoCache;
  bool m_initializedPosition;

  float m_dt;
  //added by yhz
public:
    Eigen::Vector3f m_currentPosition,m_lastPosition,m_VelSetPoint,m_AccSetPoint;
    Eigen::Vector3f m_initialPosition;
    Eigen::Vector3f m_positionRef;
    Eigen::Vector3f m_velocityRef;
    Eigen::Vector3f m_accelerationRef;
    Eigen::Vector4f m_PosSetPoint;
    Eigen::Vector3f Euler;
    Controller m_controller;
    Commander m_commander;
//    bool m_isHoverOK;
};


// handles a group of Crazyflies, which share a radio
class CrazyflieGroup
{
public:
  bool isNotSendPIng;//added by yhz
  struct latency
  {
    double objectTracking;
    double broadcasting;
  };

  CrazyflieGroup(
    const std::vector<libobjecttracker::DynamicsConfiguration>& dynamicsConfigurations,
    const std::vector<libobjecttracker::MarkerConfiguration>& markerConfigurations,
    pcl::PointCloud<pcl::PointXYZ>::Ptr pMarkers,
    std::vector<libmotioncapture::Object>* pMocapObjects,
    int radio,
    int channel,
    const std::string broadcastAddress,
    bool useMotionCaptureObjectTracking,
    const std::vector<crazyflie_driver::LogBlock>& logBlocks,
    std::string interactiveObject,
    bool writeCSVs
    )
    : m_cfs()
    , m_tracker(nullptr)
    , isNotSendPIng(false)
    , m_radio(radio)
    , m_pMarkers(pMarkers)
    , m_pMocapObjects(pMocapObjects)
    , m_slowQueue()
    , m_cfbc("radio://" + std::to_string(radio) + "/" + std::to_string(channel) + "/2M/" + broadcastAddress)
    , m_isEmergency(false)
    , m_useMotionCaptureObjectTracking(useMotionCaptureObjectTracking)
    , m_br()
    , m_interactiveObject(interactiveObject)
    , m_outputCSVs()
    , m_phase(0)
    , m_phaseStart()
    //added by yhz
    , m_sendAttSp(true)
    , m_isGivingSp(true)
    , m_initial_posMap()
    , m_formation_type("square")
    , m_formation_scale(1.5)
    , m_require_assignment(true)
//    , m_areHoverOk(false)
  {
      //added by yhz
      char str_csv[50] ;
      sprintf(str_csv,"/home/wade/ros_ws/src/crazyswarm/Cfs%d.csv",channel);
      Cf_csv.open(str_csv);printf("opened csv\n");
      std::vector<libobjecttracker::Object> objects;
      readObjects(objects, channel, logBlocks);
      std::cout<<"------ Read Object successfully --------\n"<<std::endl;
      m_tracker = new libobjecttracker::ObjectTracker(
        dynamicsConfigurations,
        markerConfigurations,
        objects);
      m_tracker->setLogWarningCallback(logWarn);
      if (writeCSVs) {
        m_outputCSVs.resize(m_cfs.size());
      }
        m_beginPosSp=true;

      /*formation control parameters and variants*/
      ros::NodeHandle nh;
      assignment_request_pub = nh.advertise<crazyflie_driver::IdPos>("/role_assignment_request",10);
      assignment_command_sub = nh.subscribe("/role_assignment_command", 10, &CrazyflieGroup::assignment_cb, this);
      m_start_time = std::chrono::high_resolution_clock::now();
      m_target_position[0] = 0.0;
      m_target_position[1] = 0.0;
      robot_number = m_cfs.size();
      m_formation.resize(2, robot_number);
      for (int i = 0; i < robot_number; ++i) {
          pair<int, int> tmp;
          tmp = make_pair(m_cfs[i]->id(),i);
          m_assignment.push_back(tmp);
      }
      cout<<"assignment : ";
      for (int i = 0; i < robot_number; ++i)
      {
      	cout<< m_assignment[i].first << "--" << m_assignment[i].second<< " ";
      }
      cout<<endl;
      /*generate formation*/
      generateFormation(m_formation_type, m_formation_scale, m_formation);
  }

  ~CrazyflieGroup()
  {
    for(auto cf : m_cfs) {
      delete cf;
    }
    delete m_tracker;
  }
  //added by yhz
  void beginPosSp()
  {
      m_beginPosSp = true;
  }
  const latency& lastLatency() const {
    return m_latency;
  }

  int radio() const {
    return m_radio;
  }

  void runInteractiveObject(std::vector<CrazyflieBroadcaster::externalPose> &states)
  {
    publishRigidBody(m_interactiveObject, 0xFF, states);
  }

  void runFast()
  {
    auto stamp = std::chrono::high_resolution_clock::now();

    std::vector<CrazyflieBroadcaster::externalPose> states;
    std::vector<CrazyflieBroadcaster::AttSetPts> sp_states;

    if (!m_interactiveObject.empty()) {
      runInteractiveObject(states);
    }

    if (m_useMotionCaptureObjectTracking) {
      for (auto cf : m_cfs) {
        publishRigidBody(cf->frame(), cf->id(), states);
      }
    } else {
      // run object tracker
      {
        auto start = std::chrono::high_resolution_clock::now();
        m_tracker->update(m_pMarkers);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedSeconds = end-start;
        m_latency.objectTracking = elapsedSeconds.count();
      }
      for (size_t i = 0; i < m_cfs.size(); ++i) {
        if (m_tracker->objects()[i].lastTransformationValid()) {
          
          const Eigen::Affine3f& transform = m_tracker->objects()[i].transformation();
          Eigen::Quaternionf q(transform.rotation());
          const auto& translation = transform.translation();

          auto rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
          // Cf_csv << rpy(0) << "," << rpy(1) << "," << rpy(2);

          if(fabs(rpy(0))>1.57){
            if(rpy(0)>1.57) {rpy(0)-=3.14;}
            else {rpy(0)+=3.14;}
          }
          
          if(fabs(rpy(1))>1.57){
            if(rpy(1)>1.57) {rpy(1)-=3.14;}
            else {rpy(1)+=3.14;}
          }
          
          if(fabs(rpy(2))>1.57){
            if(rpy(2)>1.57) {rpy(2)-=3.14;}
            else {rpy(2)+=3.14;}
          }
          if(m_cfs[i]->m_rpy(0) > -2){
            auto rpy2 = m_cfs[i]->m_rpy;

            if((fabs(rpy(0)-rpy2(0))>0.1)&&( (rpy(0)*rpy2(0))<0))
            {
              rpy(0) = -rpy(0);
            }
            if((fabs(rpy(1)-rpy2(1))>0.1)&&(rpy(1)*rpy2(1)<0))
            {
              rpy(1) = -rpy(1);
            }
            if((fabs(rpy(2)-rpy2(2))>0.1)&&(rpy(2)*rpy2(2)<0))
            {
              rpy(2) = -rpy(2);
            }
            Cf_csv << rpy2(0) << "," << rpy2(1) << "," << rpy2(2)<<",";
          }

          q = Eigen::AngleAxisf(rpy(0), Eigen::Vector3f::UnitX())
	        * Eigen::AngleAxisf(rpy(1), Eigen::Vector3f::UnitY())
	        * Eigen::AngleAxisf(rpy(2), Eigen::Vector3f::UnitZ());

          states.resize(states.size() + 1);
          states.back().id = m_cfs[i]->id();
          states.back().x = translation.x();
          states.back().y = translation.y();
          states.back().z = translation.z();

          states.back().qx = q.x();
          states.back().qy = q.y();
          states.back().qz = q.z();
          states.back().qw = q.w();

          float x,y,z;
          x = rpy(0);
          y = rpy(1);
          z = rpy(2);   
          m_cfs[i]->m_rpy(0) = x;
          m_cfs[i]->m_rpy(1) = y;
          m_cfs[i]->m_rpy(2) = z;
          
          if(isStartTr>0){
              // auto rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
              // Eigen::Quaternionf qq = Eigen::AngleAxisf(-3.0,Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(-5.0,Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(rpy(3),Eigen::Vector3f::UnitZ());
              // states.back().qx = qq.coeffs()[0];
              // states.back().qy = qq.coeffs()[1];
              // states.back().qz = qq.coeffs()[2];
              // states.back().qw = qq.coeffs()[3];
              isStartTr = -1;
            }
          /**
          vicon state prediction
          **/
          float rho = 0.6;

            if(m_sendAttSp){
                sp_states.resize(sp_states.size()+1);
                sp_states.back().id = m_cfs[i]->id();
                sp_states.back().roll = 0.0f;
                sp_states.back().pitch = 0.0f;
                sp_states.back().yaw = 0.0f;
                sp_states.back().thrust = 600.0f;
            }

          m_cfs[i]->initializePositionIfNeeded(states.back().x, states.back().y, states.back().z);

          tf::Transform tftransform;
          tftransform.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));
          tftransform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

          if (!isStopSendTr)
          {
            m_br.sendTransform(tf::StampedTransform(tftransform, ros::Time::now(), "world", m_cfs[i]->frame()));
          }

          if (m_outputCSVs.size() > 0) {
            std::chrono::duration<double> tDuration = stamp - m_phaseStart;
            double t = tDuration.count();
            *m_outputCSVs[i] << t << "," << states.back().x << "," << states.back().y << "," << states.back().z
                                  << "," << rpy(0) << "," << rpy(1) << "," << rpy(2) << "\n";
          }
          std::chrono::duration<double> tDuration = stamp - m_phaseStart;
        double t = tDuration.count();
        Cf_csv <<m_cfs[i]->id() <<","<< t << "," << states.back().x << "," << states.back().y << "," << states.back().z
                                  << "," << rpy(0) << "," << rpy(1) << "," << rpy(2)<<",";
        }
        
        else {
          std::chrono::duration<double> elapsedSeconds = stamp - m_tracker->objects()[i].lastValidTime();
          ROS_WARN("No updated pose for CF %s for %f s.",
            m_cfs[i]->frame().c_str(),
            elapsedSeconds.count());
            /*states.resize(states.size() + 1);
            states.back().id = m_cfs[i]->id();
            states.back().x = 0;
            states.back().y = 0;
            states.back().z = -1;

            states.back().qx = 0;
            states.back().qy = 0;
            states.back().qz = 0;
            states.back().qw = 0;*/


        }
      }
       Cf_csv << "\n";
    }
    {
      auto start = std::chrono::high_resolution_clock::now();
        if (m_sendAttSp){
            if(m_beginPosSp)//main process
            {
                /**
                 * add a formation control function
                 */
                auto cur_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> hover_time = cur_time-m_start_time;
                if(hover_time.count() < 6)
                {
                	// ROS_INFO("~~~~~~~Taking off");
                    for(int i=0;i<sp_states.size();++i){
                        //getGroupCurPos(states[i].id,states[i].x,states[i].y,states[i].z);
                        //getGroupCurPos(states[i].id,m_pMarkers);
                        getGroupCurPos(states[i]);
                        getPositionSetPoint();

                        CrazyflieROS *cf = m_CrazyflieIdMap[sp_states[i].id];
                        cf->m_PosSetPoint(0) = cf->m_initialPosition(0);
                        cf->m_PosSetPoint(1) = cf->m_initialPosition(1);
                        cf->m_PosSetPoint(2) = min(0.16*hover_time.count(),0.8);
                        cf->m_VelSetPoint(0) = 0;
                        cf->m_VelSetPoint(1) = 0;
                        cf->m_VelSetPoint(2) = 0.16;
                        cf->m_AccSetPoint(0) = 0;
                        cf->m_AccSetPoint(1) = 0;
                        cf->m_AccSetPoint(2) = 0;

                        Groupcontrol(sp_states[i].id,sp_states[i]);

                    }
                } else if (hover_time.count() < 38 && hover_time.count() > 35)
                {
                	ROS_INFO("~~~~~~~Landing");
                    for(int i=0;i<sp_states.size();++i){
                        //getGroupCurPos(states[i].id,states[i].x,states[i].y,states[i].z);
                        //getGroupCurPos(states[i].id,m_pMarkers);
                        getGroupCurPos(states[i]);
                        getPositionSetPoint();

                        CrazyflieROS *cf = m_CrazyflieIdMap[sp_states[i].id];
                        cf->m_PosSetPoint(0) = states[i].x;
                        cf->m_PosSetPoint(1) = states[i].y;
                        cf->m_PosSetPoint(2) = 0.5;
                        cf->m_VelSetPoint(0) = 0;
                        cf->m_VelSetPoint(1) = 0;
                        cf->m_VelSetPoint(2) = 0;
                        cf->m_AccSetPoint(0) = 0;
                        cf->m_AccSetPoint(1) = 0;
                        cf->m_AccSetPoint(2) = 0;

                        Groupcontrol(sp_states[i].id,sp_states[i]);
                    }
                } else if (hover_time.count() > 38)
                {
                    for(int i=0;i<sp_states.size();++i){
                        //getGroupCurPos(states[i].id,states[i].x,states[i].y,states[i].z);
                        //getGroupCurPos(states[i].id,m_pMarkers);
                        getGroupCurPos(states[i]);
                        getPositionSetPoint();

                        CrazyflieROS *cf = m_CrazyflieIdMap[sp_states[i].id];
                        cf->m_PosSetPoint(0) = states[i].x;
                        cf->m_PosSetPoint(1) = states[i].y;
                        cf->m_PosSetPoint(2) = -0.5;
                        cf->m_VelSetPoint(0) = 0;
                        cf->m_VelSetPoint(1) = 0;
                        cf->m_VelSetPoint(2) = 0;
                        cf->m_AccSetPoint(0) = 0;
                        cf->m_AccSetPoint(1) = 0;
                        cf->m_AccSetPoint(2) = 0;

                        Groupcontrol(sp_states[i].id,sp_states[i]);
                    }
                } else {
                    /*obtain the group position and publish for role assignment*/
                    std::vector<uint8_t> m_cfs_id;
                    std::vector<pair<double, double>> m_cfs_cur_pos;
                    std::vector<int> assignment;

                    getGroupPresentPos(states, m_cfs_id, m_cfs_cur_pos);
                    //                assignment_pub.publish(m_cfs_cur_pos);
                    assignmentReMap(m_cfs_id, m_assignment, assignment);

                    /*obtain the velocity setpoint from formation control and transform the setpoint to control variants*/
                    for (int i = 0; i < sp_states.size(); ++i) {
                        double vxy_sp[2];
                        formationControl(i, m_cfs_cur_pos, m_target_position, assignment, vxy_sp);

                        getGroupCurPos(states[i]);
                        getPositionSetPoint();

                        assignmentGroupcontrol(sp_states[i].id, sp_states[i], vxy_sp);
                    }

                    if (m_require_assignment)
                    {
                        m_require_assignment = false;
                        crazyflie_driver::IdPos msg;
                        cout<<"m_cfs_id : ";
                        for (int i = 0; i < m_cfs_id.size(); ++i) {
                        	cout<<unsigned(m_cfs_id[i])<<" ";
                            msg.id.push_back(m_cfs_id[i]);
                            msg.x.push_back(m_cfs_cur_pos[i].first);
                            msg.y.push_back(m_cfs_cur_pos[i].second);
                        }
                        cout<<endl;
                        // for (int j = 0; j < m_cfs_cur_pos.size(); ++j) {
                        //     msg.x.push_back(m_cfs_cur_pos[j].first);
                        // }
                        // for (int j = 0; j < m_cfs_cur_pos.size(); ++j) {
                        //     msg.y.push_back(m_cfs_cur_pos[j].second);
                        // }
                        assignment_request_pub.publish(msg);
                        // ROS_INFO("~~~~~~~~~request role assignment!");
                    }
                }


//                for(int i=0;i<sp_states.size();++i){
//                    //getGroupCurPos(states[i].id,states[i].x,states[i].y,states[i].z);
//                    //getGroupCurPos(states[i].id,m_pMarkers);
//                    getGroupCurPos(states[i]);
//                    getPositionSetPoint();
//
//                    Groupcontrol(sp_states[i].id,sp_states[i]);
//                }
            }
            /*std::cout<<"setpoint ouside(trpy): "<< sp_states.back().thrust <<"; "
                     <<sp_states.back().roll<<"; "
                     <<sp_states.back().pitch<<"; "
                     <<sp_states.back().yaw<<std::endl<<std::endl;*/
            m_cfbc.sendAttSps(sp_states);
        } else if (!m_sendPositionOnly) {
            m_cfbc.sendExternalPoses(states);
        }else {
            std::vector<CrazyflieBroadcaster::externalPosition> positions(states.size());
            for (size_t i = 0; i < positions.size(); ++i) {
                positions[i].id = states[i].id;
                positions[i].x  = states[i].x;
                positions[i].y  = states[i].y;
                positions[i].z  = states[i].z;
            }
            m_cfbc.sendExternalPositions(positions);
        }
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsedSeconds = end-start;
      m_latency.broadcasting = elapsedSeconds.count(); // what is this?
    }
  }

  void runSlow()
  {
    ros::NodeHandle nl("~");
    bool enableLogging;
    nl.getParam("enable_logging", enableLogging);
    
    while(ros::ok() && !m_isEmergency) {
      
      if (enableLogging && !isNotSendPIng) {
        for (const auto& cf : m_cfs) {
          cf->sendPing();
        }
      }
      m_slowQueue.callAvailable(ros::WallDuration(0));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void emergency()
  {
    m_isEmergency = true;
  }

  void takeoff(float height, float duration, uint8_t groupMask)
  {
    m_cfbc.takeoff(height, duration, groupMask);
    isNotSendPIng = true;
  }

  void land(float height, float duration, uint8_t groupMask)
  {
    isNotSendPIng = false;
      m_cfbc.land(height, duration, groupMask);
  }

  void startTrajectory(
    uint8_t trajectoryId,
    float timescale,
    bool reversed,
    uint8_t groupMask)
  {
    // for (size_t i = 0; i < 20; ++i) { //hongzhe : repeatedly send orders
      m_cfbc.startTrajectory(trajectoryId, timescale, reversed, groupMask);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }

  /**
   * Weifan
  **/
  void generateFormation(std::string formation_type, double formation_scale, Eigen::Matrix2Xd &formation)
  {
      /*Generate formation*/
      int ROBOT_MAX = this->robot_number;
      auto n1 = ROBOT_MAX - 1;
      double CIRCLE_RADIUS = formation_scale;

      // Circle case
      if (formation_type == "circle")
      {
          Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(ROBOT_MAX,0,n1).array()*(2*M_PI/ROBOT_MAX);
          formation << CIRCLE_RADIUS * t.array().cos(), CIRCLE_RADIUS * t.array().sin(); //formation
      }
      else if (formation_type == "square") // Square case
      {
          double total_length = formation_scale * 4.0;
          double x = -(formation_scale/2.0);
          double y = -(formation_scale/2.0);
          double dl = total_length / (double) ROBOT_MAX;
          double prev_l = 0;
          double dx, dy;
          for(int i = 0;i < ROBOT_MAX;i++)
          {
              double l = dl * (i+1);
              if(l < total_length/4.0)
              {
                  // go right
                  x = x + dl;
                  y = -(formation_scale/2.0);
              }
              else if(l >= total_length/4.0 && l < total_length/2.0)
              {
                  // go up
                  if(prev_l < total_length/4.0)
                  {
                      dx = formation_scale/2.0 - x;
                      dy = dl - dx;
                      x = formation_scale/2.0;
                      y = y + dy;
                  }
                  else{
                      y = y + dl;
                  }

              }
              else if(l >= total_length/2.0 && l < total_length * 3.0/4.0)
              {
                  // go left
                  if(prev_l < total_length/2.0){
                      dy = formation_scale/2.0 - y;
                      dx = dl - dy;
                      y = formation_scale/2.0;
                      x = x - dx;
                  }
                  else{
                      x = x - dl;
                  }
              }
              else{
                  // go down
                  if(prev_l < total_length * 3.0/4.0){
                      dx = x + formation_scale/2.0;
                      dy = dl - dx;
                      x = -(formation_scale/2.0);
                      y = y - dy;
                  }
                  else{
                      y = y - dl;
                  }
              }

              formation(i,0) = x;
              formation(i,1) = y;
              prev_l = l;
          }
      }
      else{
          ROS_ERROR("generate formation fails!");
      }
  }

  void getGroupPresentPos(std::vector<CrazyflieBroadcaster::externalPose> states, std::vector<uint8_t> &cfs_id, std::vector<pair<double, double>> &cfs_cur_pos)
  {
      for (int i = 0; i < states.size(); ++i) {
          cfs_id.push_back(states[i].id);
          pair<double, double> tmp;
          tmp = make_pair(states[i].x, states[i].y);
          cfs_cur_pos.push_back(tmp);
      }
  }

  void assignmentReMap(std::vector<uint8_t> cfs_id, std::vector<pair<int, int>> a_assignment, std::vector<int> &assignment)
  {
      assignment.resize(cfs_id.size());
      for (int i = 0; i < cfs_id.size(); ++i) {
          for (int j = 0; j < a_assignment.size(); ++j) {
              if (cfs_id[i] == a_assignment[j].first)
              {
                  assignment[i] = a_assignment[j].second;
                  // break;
              }
          }
      }
  }

  void formationControl(int rid, std::vector<pair<double, double>> cfs_cur_pos, double *target_position, std::vector<int> assignment, double *raw_velocity)
  {
      /**
       * Run the core control loop which does all of the heavy calculation.
       * In order to cooperate the role assignment, some adjustments need to be done
       * First, assignment matrix is unnecessary but need to replaced by the one that
       * allows two identical targets for two agents.
       * Then, self_id will be replaced by what is corresponsing.
       */
      double velocity_target_following[2],velocity_formation_control[2];
      double publish_velocity, publish_heading_velocity;

      target_following(velocity_target_following, cfs_cur_pos, target_position); //target following
      formation_control(velocity_formation_control, rid, cfs_cur_pos, m_formation, assignment);//, CONNECT_RADIUS); //establish permiter


      // ROS_INFO("*********** velocity_target_following : %f, %f", velocity_target_following[0],velocity_target_following[1]);
      // ROS_INFO("*********** velocity_formation_control : %f, %f", velocity_formation_control[0],velocity_formation_control[1]);
      double velocity[2];
      velocity[0]= velocity_target_following[0]*0.3 + velocity_formation_control[0]*0.15;
      velocity[1]= velocity_target_following[1]*0.3 + velocity_formation_control[1]*0.15;//two velocities add together
      double v_t[2];
      v_t[0] = velocity[0];
      v_t[1] = velocity[1];
      barrier_certificate(velocity, rid, cfs_cur_pos);//barrier certificate

      /*check if the role assignment is required*/
      double nominal_velocity = sqrt(v_t[0]*v_t[0]+v_t[1]*v_t[1]);
      double bc_velocity = sqrt(velocity[0]*velocity[0]+velocity[1]*velocity[1]);

      if (v_t[0]*velocity[0]+v_t[1]*velocity[1] < 0)
      {
//          ROS_INFO("warning!!!");
          velocity[0] = velocity[0]/fabs(velocity[0])*0.3;
          velocity[1] = velocity[1]/fabs(velocity[1])*0.3;
          m_require_assignment = true;
      }

//       ROS_INFO("*********** bc_velocity : %f", bc_velocity);
//       ROS_INFO("*********** nominal_velocity : %f", nominal_velocity);
//       if ((bc_velocity/2) < 0.04 && (nominal_velocity - bc_velocity) > 0.3)
//       {
//           m_require_assignment = true;
// //          assignment_request_pub.publish(signal);
      // }

      raw_velocity[0] = velocity[0];
      raw_velocity[1] = velocity[1];
  }

    void target_following(double *v, std::vector<std::pair<double, double>> position, double *target_position){
        double d[2], dv[2];
        double n;

        for (int j = 0; j < position.size(); j++)
        {
            d[0] = target_position[0] - position[j].first;
            d[1] = target_position[1] - position[j].second;

            dv[0] = d[0];
            dv[1] = d[1];
            n = n + 1;

            v[0] += dv[0];
            v[1] += dv[1];
        }
        v[0] = v[0] / (n + 1);
        v[1] = v[1] / (n + 1);// we must divide first
    }

    void formation_control(double *v, int rid, std::vector<pair<double, double>> position, Eigen::Matrix2Xd formation, std::vector<int> assignment){//, double connectivity_radius){
        // double connectivity_radius = CONNECT_RADIUS;
        int i = rid;
        double d[2], dv[2];
        double n, norm_d;
        n = 0;

        for (int j = 0; j < position.size(); j++)
        {
            if(i == j) continue;
            d[0] = position[j].first - position[i].first;
            d[1] = position[j].second - position[i].second;
            norm_d = sqrt(pow(d[0], 2) + pow(d[1], 2));

            if (norm_d < 10)//connectivity_radius)
            {
                dv[0] = d[0] - (formation(assignment[j],0) - formation(assignment[i],0));
                dv[1] = d[1] - (formation(assignment[j],1) - formation(assignment[i],1));
//                 dv[0] = d[0] - (formation(j,0) - formation(i,0));
//                 dv[1] = d[1] - (formation(j,1) - formation(i,1));
                n = n + 1;
            }
            else
            {
                dv[0] = 0;
                dv[1] = 0;
                // ROS_INFO("~~~~~~~!!!cannot see");
            }
            v[0] += dv[0];
            v[1] += dv[1];
        }
        v[0] = v[0] / (n + 1);
        v[1] = v[1] / (n + 1);// we must divide first
    }

    void barrier_certificate(double *v, int rid, std::vector<pair<double, double>> position){
        int c = 0; //c=0 assumes agressive behaviour, =1 assumes neutral, =2 assumes enemy will avoid collision
        double Ds = 0.4;
        double delta_amax = 1.0;
        double gamma = 10000.0;
        double a_max = 1.0;
        double delta_vmax = 0.5;
        double obstacle_radius = 0.5;
        double neighbour_radius = Ds + 1/(2*delta_amax)* pow((pow(2*delta_amax/gamma, 1.0/3.0) + delta_vmax), 2);

        vector<double> d0_vec;
        vector<double> d1_vec;
        vector<double> h_vec;

        double d[2], delta_v[2], d_minus[2];
        double n, norm_d;
        int i = rid;

        //Avoid the other robots
        for (int j = 0; j < position.size(); j++)
        {
            if(i == j) continue;
            d[0] = position[j].first - position[i].first;
            d[1] = position[j].second - position[i].second;
            norm_d = sqrt(pow(d[0], 2) + pow(d[1], 2));
            double v_norm = sqrt(pow(v[0], 2) + pow(v[1], 2));
            if (norm_d <= neighbour_radius)
            {
                n = n + 1;
                d0_vec.push_back(d[0]);
                d1_vec.push_back(d[1]);
                bool obstacle_avoid = false;
                d_minus[0] = -1 * d[0];
                d_minus[1] = -1 * d[1];
                delta_v[0] = v[0]/(v_norm+0.01);
                delta_v[1] = v[1]/(v_norm+0.01);
                double b = b_func(d_minus, Ds, delta_v, delta_amax, gamma, obstacle_radius, obstacle_avoid,c);
                h_vec.push_back(b);
            }
        }

        if(h_vec.size()>0){
            int k = 2;
            Eigen::VectorXd x_v = Eigen::VectorXd::Zero(k);
            Eigen::VectorXd g0(k);
            g0(0) = -2 * v[0];
            g0(1) = -2 * v[1];
            Eigen::MatrixXd G(k,k);
            G << 2, 0, 0, 2;

            d0_vec.push_back(1.0);
            d0_vec.push_back(0.0);
            d1_vec.push_back(0.0);
            d1_vec.push_back(1.0); // add matrix(1,0; 0,1)
            h_vec.push_back(a_max);
            h_vec.push_back(a_max); // add vector(a_max, a_max)

            Eigen::MatrixXd CI(k,d0_vec.size());
            Eigen::VectorXd ci0(CI.cols());

            Eigen::MatrixXd CE = Eigen::MatrixXd::Zero(k,0);
            Eigen::VectorXd ce0 = Eigen::VectorXd::Zero(CE.cols());

            for (int j = 0; j < d0_vec.size(); j++)
            {
                CI(0,j) = -1 * d0_vec[j];
                CI(1,j) = -1 * d1_vec[j];
            }

            for (int j = 0; j < h_vec.size(); j++)
            {
                ci0(j) = h_vec[j];
            }

            solve_quadprog(G, g0, CE, ce0, CI, ci0, x_v);
            v[0] = x_v(0);
            v[1] = x_v(1);
        }
    }

    double h_func(double *delta_p, double Ds, double *delta_v, double delta_amax, double obstacle_radius, bool obstacle_avoid, int c)
    {
      double delta_p_norm = sqrt(pow(delta_p[0], 2) + pow(delta_p[1], 2));
      if (obstacle_avoid){
        double h = (delta_v[0]*delta_p[0] + delta_v[1]*delta_p[1])/delta_p_norm + sqrt(c*delta_amax*(delta_p_norm - (Ds/2.0 + obstacle_radius)));
        return h;
      }
      else{
        double h = (delta_v[0]*delta_p[0] + delta_v[1]*delta_p[1])/delta_p_norm + sqrt(c*delta_amax*(delta_p_norm - Ds));
        return h;
      }
    }

    double B_func(double *delta_p, double Ds, double *delta_v, double delta_amax, double obstacle_radius, bool obstacle_avoid, int c){
      double h = h_func(delta_p, Ds, delta_v, delta_amax, obstacle_radius, obstacle_avoid, c);
      double B = 1/h;
      return B;
    }

    double b_func(double *delta_p, double Ds, double *delta_v, double delta_amax, double gamma, double obstacle_radius, bool obstacle_avoid, int c)
    {
      double B = B_func(delta_p, Ds, delta_v, delta_amax, obstacle_radius, obstacle_avoid, c);
      double h = h_func(delta_p, Ds, delta_v, delta_amax, obstacle_radius, obstacle_avoid, c);
      double delta_p_norm;
      delta_p_norm = sqrt(pow(delta_p[0], 2) + pow(delta_p[1], 2));
      double delta_v_norm = sqrt(pow(delta_v[0], 2) + pow(delta_v[1], 2));
      double amax_term = delta_amax * (delta_p[0]*delta_v[0] + delta_p[1]*delta_v[1]) / sqrt(2*delta_amax*(delta_p_norm - Ds));
      double b = gamma / B * pow(h,2) * delta_p_norm - pow((delta_p[0]*delta_v[0] + delta_p[1]*delta_v[1])/delta_p_norm,2) + pow(delta_v_norm,2) + amax_term;
      return b;
    }

    void assignmentGroupcontrol(int id,CrazyflieBroadcaster::AttSetPts& sp_state, double *vxy_sp) {
        Eigen::Vector3f Euler;
        Eigen::Vector4f vec4ftmp;
        CrazyflieROS *cf = m_CrazyflieIdMap[id];
        //std::cout<<"ID:"<<id<<std::endl;
        double z_sp = 0.8;
        cf->m_controller.control_nonLineaire(cf->m_currentPosition,
                                             z_sp, vxy_sp, cf->Euler,
                                             g_dt, &vec4ftmp);
        sp_state.roll = vec4ftmp(0);
        sp_state.pitch = vec4ftmp(1);
        sp_state.yaw = vec4ftmp(2);
        sp_state.thrust = vec4ftmp(3);
    }

    void assignment_cb(const std_msgs::Int32MultiArray &msg) {
        /*update the assignment matrix*/
        int number = int(msg.data.size()/2);
        m_assignment.clear();
        for (int i = 0; i < number; ++i) {
            std::pair<int, int> tmp;
            tmp.first = msg.data[i];
            tmp.second = msg.data[i+number];
            m_assignment.push_back(tmp);
        }
        cout<<"assignment : ";
      for (int i = 0; i < robot_number; ++i)
      {
      	cout<< m_assignment[i].first << "--" << m_assignment[i].second<< " ";
      }
      cout<<endl;
    }

  /**
  * hongzhe
  */
  void getPositionSetPoint()
  {
    ros::spinOnce();/*
    for(auto& cf : m_cfs)
    {
      cf->getSetpoint();
    }*/
  }

  void Groupcontrol(int id,CrazyflieBroadcaster::AttSetPts& sp_state) {
    Eigen::Vector3f Euler;
    Eigen::Vector4f vec4ftmp;
    CrazyflieROS *cf = m_CrazyflieIdMap[id];
    //std::cout<<"ID:"<<id<<std::endl;
    cf->m_controller.control_nonLineaire(cf->m_currentPosition,
                                                    cf->m_PosSetPoint, cf->m_VelSetPoint, cf->m_AccSetPoint, cf->Euler,
                                                    g_dt, &vec4ftmp);
    sp_state.roll = vec4ftmp(0);
    sp_state.pitch = vec4ftmp(1);
    sp_state.yaw = vec4ftmp(2);
    sp_state.thrust = vec4ftmp(3);
  }

  void getGroupCurPos(uint8_t id, float x, float y, float z)
  {
      CrazyflieROS *cf = m_CrazyflieIdMap[id];
      cf->giveCurrentPos(x,y,z);
  }


  void getGroupCurPos(CrazyflieBroadcaster::externalPose state)
  {
      CrazyflieROS *cf = m_CrazyflieIdMap[state.id];
      cf->giveCurrentPos(state.x,state.y,state.z);
      Eigen::Quaternionf q(state.qw,state.qx,state.qy,state.qz);
      auto rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
      cf->Euler(0)=rpy(0);
      cf->Euler(1)=rpy(1);
      cf->Euler(2)=rpy(2);
  }

  void getGroupCurPos(uint8_t id, pcl::PointCloud<pcl::PointXYZ>::Ptr pmarkers)
  {
      CrazyflieROS *cf = m_CrazyflieIdMap[id];
      cf->giveCurrentPos(pmarkers);
  }
  /**
   * end
   */

  void nextPhase()
  {
      for (size_t i = 0; i < m_outputCSVs.size(); ++i) {
        auto& file = *m_outputCSVs[i];
        // file.close();
        file.open("cf" + std::to_string(m_cfs[i]->id()) + "_phase" + std::to_string(m_phase + 1) + ".csv");
        file << "t,x,y,z,roll,pitch,yaw\n";
      }
      m_phase += 1;
      m_phaseStart = std::chrono::system_clock::now();
  }
#if 0
  template<class T, class U>
  void updateParam(uint8_t group, uint8_t id, Crazyflie::ParamType type, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cfbc.setParam<T>(group, id, type, (T)value);
  }

  void updateParams(
    uint8_t group,
    const std::vector<std::string>& params)
  {
    for (const auto& p : params) {
      std::string ros_param = "/cfgroup" + std::to_string((int)group) + "/" + p;
      size_t pos = p.find("/");
      std::string g(p.begin(), p.begin() + pos);
      std::string n(p.begin() + pos + 1, p.end());

      // TODO: this assumes that all IDs are identically
      //       should use byName lookup instead!
      auto entry = m_cfs.front()->getParamTocEntry(g, n);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(group, entry->id, entry->type, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(group, entry->id, entry->type, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(group, entry->id, entry->type, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(group, entry->id, entry->type, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(group, entry->id, entry->type, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(group, entry->id, entry->type, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(group, entry->id, entry->type, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", g.c_str(), n.c_str());
      }
    }
  }
#endif



private:
  std::ofstream Cf_csv;
  void publishRigidBody(const std::string& name, uint8_t id, std::vector<CrazyflieBroadcaster::externalPose> &states)
  {
    bool found = false;
    for (const auto& rigidBody : *m_pMocapObjects) {
      if (   rigidBody.name() == name
          && !rigidBody.occluded()) {

        states.resize(states.size() + 1);
        states.back().id = id;
        states.back().x = rigidBody.position().x();
        states.back().y = rigidBody.position().y();
        states.back().z = rigidBody.position().z();
        states.back().qx = rigidBody.rotation().x();
        states.back().qy = rigidBody.rotation().y();
        states.back().qz = rigidBody.rotation().z();
        states.back().qw = rigidBody.rotation().w();

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(
          states.back().x,
          states.back().y,
          states.back().z));
        tf::Quaternion q(
          states.back().qx,
          states.back().qy,
          states.back().qz,
          states.back().qw);
        transform.setRotation(q);
        m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
        found = true;
        break;
      }

    }

    if (!found) {
      ROS_WARN("No updated pose for motion capture object %s", name.c_str());
    }
  }


  void readObjects(
    std::vector<libobjecttracker::Object>& objects,
    int channel,
    const std::vector<crazyflie_driver::LogBlock>& logBlocks)
  {
    // read CF config
    struct CFConfig
    {
      std::string uri;
      std::string tf_prefix;
      std::string frame;
      int idNumber;
      std::string type;
    };

    ros::NodeHandle nGlobal;

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);
      std::cout<<"------ Read Object successfully --------\n"<<std::endl;

    objects.clear();
    m_cfs.clear();
    std::vector<CFConfig> cfConfigs;
    for (int32_t i = 0; i < crazyflies.size(); ++i) {
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int id = crazyflie["id"];
      int ch = crazyflie["channel"];
      std::string type = crazyflie["type"];

        if (ch == channel) {
        XmlRpc::XmlRpcValue pos = crazyflie["initialPosition"];
        ROS_ASSERT(pos.getType() == XmlRpc::XmlRpcValue::TypeArray);
        std::cout<<"CF"<<id<<"------ Read Object successfully 1--------\n"<<std::endl;

        std::vector<double> posVec(3);
        for (int32_t j = 0; j < pos.size(); ++j) {
          ROS_ASSERT(pos[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          double f = static_cast<double>(pos[j]);
          posVec[j] = f;
        }
        Eigen::Affine3f m;
        m = Eigen::Translation3f(posVec[0], posVec[1], posVec[2]);
        Eigen::Vector3f initpos(posVec[0], posVec[1], posVec[2]);
        m_initial_posMap[id] = initpos;

        std::cout<<"init pos:"<< std::move(m_initial_posMap[id])[0]<<" "<<std::move(m_initial_posMap[id])[1]<<" "
                                                                   <<std::move(m_initial_posMap[id])[2]<<endl;
        int markerConfigurationIdx;
        nGlobal.getParam("crazyflieTypes/" + type + "/markerConfiguration", markerConfigurationIdx);
        int dynamicsConfigurationIdx;
        nGlobal.getParam("crazyflieTypes/" + type + "/dynamicsConfiguration", dynamicsConfigurationIdx);
        objects.push_back(libobjecttracker::Object(markerConfigurationIdx, dynamicsConfigurationIdx, m));

        std::stringstream sstr;
        sstr << std::setfill ('0') << std::setw(2) << std::hex << id;
        std::string idHex = sstr.str();
        
        std::string uri = "radio://" + std::to_string(m_radio) + "/" + std::to_string(channel) + "/2M/E7E7E7E7" + idHex;
        // std::string uri = "radio://" + std::to_string(m_radio) + "/" + std::to_string(channel) + "/2M/E7" + idHex + idHex + idHex + idHex;
        std::string tf_prefix = "cf" + std::to_string(id);
        std::string frame = "cf" + std::to_string(id);
        cfConfigs.push_back({uri, tf_prefix, frame, id, type});
      }
    }
    

    // Turn all CFs on
    for (const auto& config : cfConfigs) {
      Crazyflie cf(config.uri);
      cf.syson();
      for (size_t i = 0; i < 100; ++i) {
        cf.sendPing();
      }
    }

    ros::NodeHandle nl("~");
    bool enableLogging;
    bool enableParameters;
    bool forceNoCache;

    nl.getParam("enable_logging", enableLogging);
    nl.getParam("enable_parameters", enableParameters);
    nl.getParam("force_no_cache", forceNoCache);

    // add Crazyflies
    for (const auto& config : cfConfigs) {
      addCrazyflie(config.uri, config.tf_prefix, config.frame, "/world", enableParameters, enableLogging, config.idNumber, config.type, logBlocks, forceNoCache);

      auto start = std::chrono::high_resolution_clock::now();
      updateParams(m_cfs.back());
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;
      ROS_INFO("Update params: %f s", elapsed.count());
    }
  }

  void addCrazyflie(
    const std::string& uri,
    const std::string& tf_prefix,
    const std::string& frame,
    const std::string& worldFrame,
    bool enableParameters,
    bool enableLogging,
    int id,
    const std::string& type,
    const std::vector<crazyflie_driver::LogBlock>& logBlocks,
    bool forceNoCache)
  {
    ROS_INFO("Adding CF: %s (%s, %s)...", tf_prefix.c_str(), uri.c_str(), frame.c_str());
    auto start = std::chrono::high_resolution_clock::now();
    CrazyflieROS* cf = new CrazyflieROS(
      uri,
      tf_prefix,
      frame,
      worldFrame,
      enableParameters,
      enableLogging,
      id,
      type,
      logBlocks,
      m_slowQueue,
      forceNoCache);
    m_CrazyflieIdMap[id] = cf;
      ROS_INFO("--------- debug 1------------\n");
      cf->setTakeOffPos(m_initial_posMap[id][0], m_initial_posMap[id][1], m_initial_posMap[id][2]);
      ROS_INFO("--------- debug 2------------\n");
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO("CF ctor: %f s", elapsed.count());
    cf->run(m_slowQueue);
    auto end2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed2 = end2 - end;
    ROS_INFO("CF run: %f s", elapsed2.count());
    m_cfs.push_back(cf);
  }

  void updateParams(
    CrazyflieROS* cf)
  {
    ros::NodeHandle n("~");
    ros::NodeHandle nGlobal;
    // update parameters
    // std::cout << "attempt: " << "firmwareParams/" + cf->type() << std::endl;
    // char dummy;
    // std::cin >> dummy;

    // update global and type-specific parameters
    std::vector<std::string> paramLocations;
    paramLocations.push_back("firmwareParams");
    paramLocations.push_back("crazyflieTypes/" + cf->type() + "/firmwareParams");

    crazyflie_driver::UpdateParams::Request request;
    crazyflie_driver::UpdateParams::Response response;

    for (const auto& paramLocation : paramLocations) {
      XmlRpc::XmlRpcValue firmwareParams;
      if (paramLocation == "firmwareParams") {
        n.getParam(paramLocation, firmwareParams);
      } else {
        nGlobal.getParam(paramLocation, firmwareParams);
      }

      // ROS_ASSERT(firmwareParams.getType() == XmlRpc::XmlRpcValue::TypeArray);
      auto iter = firmwareParams.begin();
      for (; iter != firmwareParams.end(); ++iter) {
        std::string group = iter->first;
        XmlRpc::XmlRpcValue v = iter->second;
        auto iter2 = v.begin();
        for (; iter2 != v.end(); ++iter2) {
          std::string param = iter2->first;
          XmlRpc::XmlRpcValue value = iter2->second;
          if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            bool b = value;
            nGlobal.setParam(cf->frame() + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            int b = value;
            nGlobal.setParam(cf->frame() + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            double b = value;
            nGlobal.setParam(cf->frame() + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } else {
            ROS_WARN("No known type for %s.%s!", group.c_str(), param.c_str());
          }
          request.params.push_back(group + "/" + param);

        }
      }
    }
    cf->updateParams(request, response);
  }

private:
    /*formation control parameters and variants*/
    std::string m_formation_type;
    double m_target_position[2];
    double m_formation_scale;
    Eigen::Matrix2Xd m_formation;
    std::vector<pair<int,int>> m_assignment;
    int robot_number;
    std::chrono::high_resolution_clock::time_point m_start_time;
    ros::Publisher assignment_request_pub;
    ros::Subscriber assignment_command_sub;
    bool m_require_assignment;
//    bool m_areHoverOk;

    std::vector<CrazyflieROS*> m_cfs;
    std::string m_interactiveObject;
    libobjecttracker::ObjectTracker* m_tracker;
    int m_radio;
    // ViconDataStreamSDK::CPP::Client* m_pClient;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pMarkers;
    std::vector<libmotioncapture::Object>* m_pMocapObjects;
    ros::CallbackQueue m_slowQueue;
    CrazyflieBroadcaster m_cfbc;
    bool m_isEmergency;
    bool m_useMotionCaptureObjectTracking;
    tf::TransformBroadcaster m_br;
    latency m_latency;
    bool m_sendPositionOnly, m_sendAttSp, m_isGivingSp;
    std::vector<std::unique_ptr<std::ofstream>> m_outputCSVs;
    int m_phase;
    std::chrono::high_resolution_clock::time_point m_phaseStart;
    std::map<uint8_t, CrazyflieROS*> m_CrazyflieIdMap;
    std::map<uint8_t, Eigen::Vector3f> m_initial_posMap;

public:
    bool m_beginPosSp;
};

// handles all Crazyflies
class CrazyflieServer
{
public:
  CrazyflieServer()
    : m_isEmergency(false)
    , m_serviceEmergency()
    , m_serviceStartTrajectory()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_serviceNextPhase()
    , m_serviceGetPosSetPoint()
    , m_lastInteractiveObjectPosition(-10, -10, 1)
    , m_broadcastingNumRepeats(15)
    , m_broadcastingDelayBetweenRepeatsMs(1)

  {
    
    ros::NodeHandle nh;
    nh.setCallbackQueue(&m_queue);
    
    m_serviceEmergency = nh.advertiseService("emergency", &CrazyflieServer::emergency, this);
    
    m_serviceStartTrajectory = nh.advertiseService("start_trajectory", &CrazyflieServer::startTrajectory, this);
    m_serviceTakeoff = nh.advertiseService("takeoff", &CrazyflieServer::takeoff, this);
    m_serviceLand = nh.advertiseService("land", &CrazyflieServer::land, this);
    m_serviceNextPhase = nh.advertiseService("next_phase", &CrazyflieServer::nextPhase, this);
    // m_serviceUpdateParams = nh.advertiseService("update_params", &CrazyflieServer::updateParams, this);
    m_pubPointCloud = nh.advertise<sensor_msgs::PointCloud>("pointCloud", 1);
    m_serviceGetPosSetPoint = nh.advertiseService("getServerPosSetPoint",&CrazyflieServer::getPosSetPoint, this);
    m_serviceTrajectoryRef=nh.advertiseService("/set_trajectory_ref", &CrazyflieServer::setTrajectoryRef, this);
    gPositionRef.setZero();
    gPositionRef(2)=-1;

    // m_subscribeVirtualInteractiveObject = nh.subscribe("virtual_interactive_object", 1, &CrazyflieServer::virtualInteractiveObjectCallback, this);

  }

  ~CrazyflieServer()
  {
    for (CrazyflieGroup* group : m_groups) {
      delete group;
    }
  }

  void virtualInteractiveObjectCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    m_lastInteractiveObjectPosition = Eigen::Vector3f(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);
  }



    bool setTrajectoryRef(
            crazyflie_driver::TrajectoryRef::Request& req,
            crazyflie_driver::TrajectoryRef::Response& res)
    {
      //ROS_INFO("[%s] Set Group Mask", m_frame.c_str());

      gPositionRef(0)=req.x;
      gPositionRef(1)=req.y;
      gPositionRef(2)=req.z;
      return true;
    }


  void run()
  {
    std::thread tSlow(&CrazyflieServer::runSlow, this);
    runFast();
    tSlow.join();
  }

  void runFast()
  {
    
    // std::vector<CrazyflieBroadcaster::externalPose> states(1);
    // states.back().id = 07;
    // states.back().q0 = 0;
    // states.back().q1 = 0;
    // states.back().q2 = 0;
    // states.back().q3 = 1;


    // while(ros::ok()) {

    //   m_cfbc.sendPositionExternalBringup(states);
    //   // m_cfs[0]->sendPositionExternalBringup(states[0]);
    //   m_fastQueue.callAvailable(ros::WallDuration(0));
    //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
    // return;

    std::vector<libobjecttracker::DynamicsConfiguration> dynamicsConfigurations;
    std::vector<libobjecttracker::MarkerConfiguration> markerConfigurations;
    std::set<int> channels;

    readMarkerConfigurations(markerConfigurations);
    readDynamicsConfigurations(dynamicsConfigurations);
    readChannels(channels);

    std::string broadcastAddress;
    bool useMotionCaptureObjectTracking;
    std::string logFilePath;
    std::string interactiveObject;
    bool printLatency;
    bool writeCSVs;
    bool sendPositionOnly;
    std::string motionCaptureType;

    ros::NodeHandle nl("~");
    std::string objectTrackingType;
    nl.getParam("object_tracking_type", objectTrackingType);
    useMotionCaptureObjectTracking = (objectTrackingType == "motionCapture");
    nl.getParam("broadcast_address", broadcastAddress);
    nl.param<std::string>("save_point_clouds", logFilePath, "");
    nl.param<std::string>("interactive_object", interactiveObject, "");
    nl.getParam("print_latency", printLatency);
    nl.getParam("write_csvs", writeCSVs);
    nl.param<std::string>("motion_capture_type", motionCaptureType, "vicon");

    nl.param<int>("broadcasting_num_repeats", m_broadcastingNumRepeats, 15);
    nl.param<int>("broadcasting_delay_between_repeats_ms", m_broadcastingDelayBetweenRepeatsMs, 1);

    // tilde-expansion
    wordexp_t wordexp_result;
    if (wordexp(logFilePath.c_str(), &wordexp_result, 0) == 0) {
      // success - only read first result, could be more if globs were used
      logFilePath = wordexp_result.we_wordv[0];
    }
    wordfree(&wordexp_result);
    
    libobjecttracker::PointCloudLogger pointCloudLogger(logFilePath);
    const bool logClouds = !logFilePath.empty();

    // custom log blocks
    std::vector<std::string> genericLogTopics;
    nl.param("genericLogTopics", genericLogTopics, std::vector<std::string>());
    std::vector<int> genericLogTopicFrequencies;
    nl.param("genericLogTopicFrequencies", genericLogTopicFrequencies, std::vector<int>());
    std::vector<crazyflie_driver::LogBlock> logBlocks;

    if (genericLogTopics.size() == genericLogTopicFrequencies.size())
    {
      size_t i = 0;
      for (auto& topic : genericLogTopics)
      {
        crazyflie_driver::LogBlock logBlock;
        logBlock.topic_name = topic;
        logBlock.frequency = genericLogTopicFrequencies[i];
        nl.getParam("genericLogTopic_" + topic + "_Variables", logBlock.variables);
        logBlocks.push_back(logBlock);
        ++i;
      }
    }
    else
    {
      ROS_ERROR("Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!");
    }

    // Make a new client
    libmotioncapture::MotionCapture* mocap = nullptr;
    
    if (false)
    {
    }
#ifdef ENABLE_VICON
    else if (motionCaptureType == "vicon")
    {
      std::string hostName;
      nl.getParam("vicon_host_name", hostName);
      mocap = new libmotioncapture::MotionCaptureVicon(hostName,
        /*enableObjects*/useMotionCaptureObjectTracking || !interactiveObject.empty(),
        /*enablePointcloud*/ !useMotionCaptureObjectTracking);
    }
#endif
#ifdef ENABLE_OPTITRACK
    else if (motionCaptureType == "optitrack")
    {
      std::string localIP;
      std::string serverIP;
      nl.getParam("optitrack_local_ip", localIP);
      nl.getParam("optitrack_server_ip", serverIP);
      mocap = new libmotioncapture::MotionCaptureOptitrack(localIP, serverIP);
    }
#endif
#ifdef ENABLE_PHASESPACE
    else if (motionCaptureType == "phasespace")
    {
      std::string ip;
      int numMarkers;
      nl.getParam("phasespace_ip", ip);
      nl.getParam("phasespace_num_markers", numMarkers);
      std::map<size_t, std::pair<int, int> > cfs;
      cfs[231] = std::make_pair<int, int>(10, 11);
      mocap = new libmotioncapture::MotionCapturePhasespace(ip, numMarkers, cfs);
    }
#endif
    else {
      throw std::runtime_error("Unknown motion capture type!");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<libmotioncapture::Object> mocapObjects;

  // Create all groups in parallel and launch threads
    {
      std::vector<std::future<CrazyflieGroup*> > handles;
      int r = 0;
      std::cout << "ch: " << channels.size() << std::endl;
      for (int channel : channels) {
        auto handle = std::async(std::launch::async,
                                 [&](int channel, int radio)
                                 {
                                     // std::cout << "radio: " << radio << std::endl;
                                     return new CrazyflieGroup(
                                             dynamicsConfigurations,
                                             markerConfigurations,
                                             // &client,
                                             markers,
                                             &mocapObjects,
                                             radio,
                                             channel,
                                             broadcastAddress,
                                             useMotionCaptureObjectTracking,
                                             logBlocks,
                                             interactiveObject,
                                             writeCSVs);
                                 },
                                 channel,
                                 r
        );
        handles.push_back(std::move(handle));
        ++r;
        printf("-------------run fast here ------------------\n");
        std::cout<<"handles: "<<handles.size()<<std::endl;
      }

      for (auto& handle : handles) {
          printf("-------------run fast here 1 groups: %d------------------\n",m_groups.size());
          m_groups.push_back(handle.get());
      }
    }
  
    // start the groups threads
    std::vector<std::thread> threads;
    for (auto& group : m_groups) {
      threads.push_back(std::thread(&CrazyflieGroup::runSlow, group));
    }

    ROS_INFO("Started %lu threads", threads.size());

    // Connect to a server
    // ROS_INFO("Connecting to %s ...", hostName.c_str());
    // while (ros::ok() && !client.IsConnected().Connected) {
    //   // Direct connection
    //   bool ok = (client.Connect(hostName).Result == Result::Success);
    //   if(!ok) {
    //     ROS_WARN("Connect failed...");
    //   }
    //   ros::spinOnce();
    // }

    // setup messages
    sensor_msgs::PointCloud msgPointCloud;
    msgPointCloud.header.seq = 0;
    msgPointCloud.header.frame_id = "world";

    auto startTime = std::chrono::high_resolution_clock::now();

    struct latencyEntry {
      std::string name;
      double secs;
    };
    std::vector<latencyEntry> latencies;

    std::vector<double> latencyTotal(6 + 3 * 2, 0.0);
    uint32_t latencyCount = 0;
    std::vector<libmotioncapture::LatencyInfo> mocapLatency;

    while (ros::ok() && !m_isEmergency) {
      // Get a frame
      mocap->waitForNextFrame();
      g_dt=mocap->getTimeIncrement();
      //mocap->waitForNextFrame();
      //g_dt+=mocap->getTimeIncrement();


      //std::cout<<"g_dt:"<<g_dt<<endl;

      latencies.clear();

      auto startIteration = std::chrono::high_resolution_clock::now();
      double totalLatency = 0;

      // Get the latency
      mocap->getLatency(mocapLatency);
      float viconLatency = 0;
      for (const auto& item : mocapLatency) {
        viconLatency += item.value();
      }
      if (viconLatency > 0.035) {
        std::stringstream sstr;
        sstr << "VICON Latency high: " << viconLatency << " s." << std::endl;
        for (const auto& item : mocapLatency) {
          sstr << "  Latency: " << item.name() << ": " << item.value() << " s." << std::endl;
        }
        ROS_WARN("%s", sstr.str().c_str());
      }

      if (printLatency) {
        size_t i = 0;
        for (const auto& item : mocapLatency) {
          latencies.push_back({item.name(), item.value()});
          latencyTotal[i] += item.value();
          totalLatency += item.value();
          latencyTotal.back() += item.value();
        }
        ++i;
      }

      // size_t latencyCount = client.GetLatencySampleCount().Count;
      // for(size_t i = 0; i < latencyCount; ++i) {
      //   std::string sampleName  = client.GetLatencySampleName(i).Name;
      //   double      sampleValue = client.GetLatencySampleValue(sampleName).Value;

      //   ROS_INFO("Latency: %s: %f", sampleName.c_str(), sampleValue);
      // }

      // Get the unlabeled markers and create point cloud
      if (!useMotionCaptureObjectTracking) {
  //          std::cout<<"---------- de ------------"<<std::endl;
          mocap->getPointCloud(markers);
  //        std::cout<<"----- size: --------"<<markers->size()<<std::endl;
        msgPointCloud.header.seq += 1;
        msgPointCloud.header.stamp = ros::Time::now();
        msgPointCloud.points.resize(markers->size());
        for (size_t i = 0; i < markers->size(); ++i) {
          const pcl::PointXYZ& point = markers->at(i);
          msgPointCloud.points[i].x = point.x;
          msgPointCloud.points[i].y = point.y;
          msgPointCloud.points[i].z = point.z;
        }
        m_pubPointCloud.publish(msgPointCloud);

        if (logClouds) {
          pointCloudLogger.log(markers);
        }
      }

      if (useMotionCaptureObjectTracking || !interactiveObject.empty()) {
        // get mocap rigid bodies
        mocapObjects.clear();
        mocap->getObjects(mocapObjects);
        if (interactiveObject == "virtual") {
          Eigen::Quaternionf quat(0, 0, 0, 1);
          mocapObjects.push_back(
            libmotioncapture::Object(
              interactiveObject,
              m_lastInteractiveObjectPosition,
              quat));
        }
      }

      auto startRunGroups = std::chrono::high_resolution_clock::now();
      std::vector<std::future<void> > handles;
      for (auto group : m_groups) {
        auto handle = std::async(std::launch::async, &CrazyflieGroup::runFast, group);
        handles.push_back(std::move(handle));
      }

      for (auto& handle : handles) {
        handle.wait();
      }
      auto endRunGroups = std::chrono::high_resolution_clock::now();
      if (printLatency) {
        std::chrono::duration<double> elapsedRunGroups = endRunGroups - startRunGroups;
        latencies.push_back({"Run All Groups", elapsedRunGroups.count()});
        latencyTotal[4] += elapsedRunGroups.count();
        totalLatency += elapsedRunGroups.count();
        latencyTotal.back() += elapsedRunGroups.count();
        int groupId = 0;
        for (auto group : m_groups) {
          auto latency = group->lastLatency();
          int radio = group->radio();
          latencies.push_back({"Group " + std::to_string(radio) + " objectTracking", latency.objectTracking});
          latencies.push_back({"Group " + std::to_string(radio) + " broadcasting", latency.broadcasting});
          latencyTotal[5 + 2*groupId] += latency.objectTracking;
          latencyTotal[6 + 2*groupId] += latency.broadcasting;
          ++groupId;
        }
      }

      auto endIteration = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = endIteration - startIteration;
      double elapsedSeconds = elapsed.count();
      if (elapsedSeconds > 0.009) {
        ROS_WARN("Latency too high! Is %f s.", elapsedSeconds);
      }

      if (printLatency) {
        ++latencyCount;
        std::cout << "Latencies" << std::endl;
        for (auto& latency : latencies) {
          std::cout << latency.name << ": " << latency.secs * 1000 << " ms" << std::endl;
        }
        std::cout << "Total " << totalLatency * 1000 << " ms" << std::endl;
        // // if (latencyCount % 100 == 0) {
          std::cout << "Avg " << latencyCount << std::endl;
          for (size_t i = 0; i < latencyTotal.size(); ++i) {
            std::cout << latencyTotal[i] / latencyCount * 1000.0 << ",";
          }
          std::cout << std::endl;
        // // }
      }

        // ROS_INFO("Latency: %f s", elapsedSeconds.count());

        // m_fastQueue.callAvailable(ros::WallDuration(0));
    }

    if (logClouds) {
      pointCloudLogger.flush();
    }

    // wait for other threads
    for (auto& thread : threads) {
      thread.join();
    }
  }

  void runSlow()
  {
    while(ros::ok() && !m_isEmergency) {
      m_queue.callAvailable(ros::WallDuration(0));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:
  
  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    for (auto& group : m_groups) {
      group->emergency();
    }
    m_isEmergency = true;

    return true;
  }

  bool takeoff(
    crazyflie_driver::Takeoff::Request& req,
    crazyflie_driver::Takeoff::Response& res)
  {
    ROS_INFO("Takeoff!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      int countg = 0;
      for (auto& group : m_groups) {
        group->takeoff(req.height, req.duration.toSec(), req.groupMask);
        // group->takeoff(0.8f, req.duration.toSec(), req.groupMask);
        ++countg;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }

    return true;
  }

  bool land(
    crazyflie_driver::Land::Request& req,
    crazyflie_driver::Land::Response& res)
  {
    ROS_INFO("Land!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto& group : m_groups) {
        group->land(req.height, req.duration.toSec(), req.groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }

    return true;
  }

  bool startTrajectory(
    crazyflie_driver::StartTrajectory::Request& req,
    crazyflie_driver::StartTrajectory::Response& res)
  {
    ROS_INFO("Start trajectory!");
    if(isStartTr ==0){
      isStartTr = 1;
    } 
    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto& group : m_groups) {
        group->startTrajectory(req.trajectoryId, req.timescale, req.reversed, req.groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    return true;
  }
  bool getPosSetPoint(
          crazyflie_driver::getPosSetPoint::Request& req,
          crazyflie_driver::getPosSetPoint::Response& res)
  {
      for(auto& group : m_groups){
          group->beginPosSp();
      }

      return true;
  }

  bool nextPhase(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_INFO("NextPhase!");
    for (auto& group : m_groups) {
      group->nextPhase();
    }

    return true;
  }
#if 0
  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("UpdateParams!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto& group : m_groups) {
        group->updateParams(req.group, req.params);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }

    return true;
  }
#endif
//
  void readMarkerConfigurations(
    std::vector<libobjecttracker::MarkerConfiguration>& markerConfigurations)
  {
    markerConfigurations.clear();
    ros::NodeHandle nGlobal;
    int numConfigurations;
    nGlobal.getParam("numMarkerConfigurations", numConfigurations);
    for (int i = 0; i < numConfigurations; ++i) {
      markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
      std::stringstream sstr;
      sstr << "markerConfigurations/" << i << "/numPoints";
      int numPoints;
      nGlobal.getParam(sstr.str(), numPoints);

      std::vector<double> offset;
      std::stringstream sstr2;
      sstr2 << "markerConfigurations/" << i << "/offset";
      nGlobal.getParam(sstr2.str(), offset);
      for (int j = 0; j < numPoints; ++j) {
        std::stringstream sstr3;
        sstr3 << "markerConfigurations/" << i << "/points/" << j;
        std::vector<double> points;
        nGlobal.getParam(sstr3.str(), points);
        markerConfigurations.back()->push_back(pcl::PointXYZ(points[0] + offset[0], points[1] + offset[1], points[2] + offset[2]));
      }
    }
  }

  void readDynamicsConfigurations(
    std::vector<libobjecttracker::DynamicsConfiguration>& dynamicsConfigurations)
  {
    ros::NodeHandle nGlobal;
    int numConfigurations;
    nGlobal.getParam("numDynamicsConfigurations", numConfigurations);
    dynamicsConfigurations.resize(numConfigurations);
    for (int i = 0; i < numConfigurations; ++i) {
      std::stringstream sstr;
      sstr << "dynamicsConfigurations/" << i;
      nGlobal.getParam(sstr.str() + "/maxXVelocity", dynamicsConfigurations[i].maxXVelocity);
      nGlobal.getParam(sstr.str() + "/maxYVelocity", dynamicsConfigurations[i].maxYVelocity);
      nGlobal.getParam(sstr.str() + "/maxZVelocity", dynamicsConfigurations[i].maxZVelocity);
      nGlobal.getParam(sstr.str() + "/maxPitchRate", dynamicsConfigurations[i].maxPitchRate);
      nGlobal.getParam(sstr.str() + "/maxRollRate", dynamicsConfigurations[i].maxRollRate);
      nGlobal.getParam(sstr.str() + "/maxYawRate", dynamicsConfigurations[i].maxYawRate);
      nGlobal.getParam(sstr.str() + "/maxRoll", dynamicsConfigurations[i].maxRoll);
      nGlobal.getParam(sstr.str() + "/maxPitch", dynamicsConfigurations[i].maxPitch);
      nGlobal.getParam(sstr.str() + "/maxFitnessScore", dynamicsConfigurations[i].maxFitnessScore);
    }
  }

  void readChannels(
    std::set<int>& channels)
  {
    // read CF config
    ros::NodeHandle nGlobal;

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

    channels.clear();
    for (int32_t i = 0; i < crazyflies.size(); ++i) {
      
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int channel = crazyflie["channel"];
      channels.insert(channel);
      // std::cout<<channel<<std::endl;
    }
  }

private:
  std::string m_worldFrame;
  bool m_isEmergency;
  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceStartTrajectory;
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  ros::ServiceServer m_serviceNextPhase;
  ros::ServiceServer m_serviceUpdateParams;
  ros::ServiceServer m_serviceGetPosSetPoint;
  ros::ServiceServer m_serviceTrajectoryRef;

  ros::Publisher m_pubPointCloud;
  // tf::TransformBroadcaster m_br;

  std::vector<CrazyflieGroup*> m_groups;

  ros::Subscriber m_subscribeVirtualInteractiveObject;
  Eigen::Vector3f m_lastInteractiveObjectPosition;

  int m_broadcastingNumRepeats;
  int m_broadcastingDelayBetweenRepeatsMs;

private:
  // We have two callback queues
  // 1. Fast queue handles pose and emergency callbacks. Those are high-priority and can be served quickly
  // 2. Slow queue handles all other requests.
  // Each queue is handled in its own thread. We don't want a thread per CF to make sure that the fast queue
  //  gets called frequently enough.

  ros::CallbackQueue m_queue;
  // ros::CallbackQueue m_slowQueue;
};

int main(int argc, char **argv)
{
  // raise(SIGSTOP);

  ros::init(argc, argv, "crazyswarm_server");

  // ros::NodeHandle n("~");
  // std::string worldFrame;
  // n.param<std::string>("world_frame", worldFrame, "/world");
  // std::string broadcastUri;
  // n.getParam("broadcast_uri", broadcastUri);

  CrazyflieServer server;//(broadcastUri, worldFrame);

  // read CF config
  ros::NodeHandle nGlobal;

  XmlRpc::XmlRpcValue crazyflies;
  nGlobal.getParam("crazyflies", crazyflies);
  ROS_INFO("assert %d",crazyflies.getType());
  ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

  std::set<int> cfIds;
  for (int32_t i = 0; i < crazyflies.size(); ++i)
  {
    ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
    int id = crazyflie["id"];
    int channel = crazyflie["channel"];
    // printf("----------------%d------------------\n",id);
    if (cfIds.find(id) != cfIds.end()) {
      ROS_FATAL("CF with the same id twice in configuration!");
      return 1;
    }
    cfIds.insert(id);
  }

  // ROS_INFO("All CFs are ready!");

  server.run();

  return 0;
}
