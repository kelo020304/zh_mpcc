#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <vector>

using namespace std;

const double PI = 3.1415926;
static std::string ODOM_TOPIC;  //将状态估计信息话题写为参数在launch中加载
double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;           // 发布次数
int pubSkipCount = 0;         // 发布次数计数器
bool twoWayDrive = true;      // 双向驱动
double lookAheadDis = 0.5;    // 向前搜索的路点距离
double lookAheadSpeedGain = 0.0;
double yawRateGain = 7.5;     // 普通的yaw率增益
double stopYawRateGain = 7.5; // 停止的yaw率增益
double maxYawRate = 45.0;     // 最大旋转速度
bool useYawRateFilter = true;
double yawRateFilterAlpha = 0.3;
bool useYawRateRateLimit = true;
double yawRateMaxDelta = 10.0;
double maxSpeed = 1.0;        // 最大直线速度
double maxAccel = 1.0;        // 最大加速度
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;       // 方向差阈值
double dirDiffGoThre = 0.08;    // 小于该阈值才允许前进
double dirDiffStopThre = 0.18;  // 大于该阈值才强制停止
double dirDiffTurnOnlyThre = 0.45; // 大于该阈值只原地转向
double turnOnlyDistThre = 0.6;  // 小于该距离才允许纯原地转向
double stopDisThre = 0.2;       // 停止距离阈值
double slowDwnDisThre = 1.0;    // 减速距离阈值
double posOnlyDistThre = 0.5;   // 小于该距离仅做角度调整
double nearGoalNoSwitchDist = 2.5; // 近目标不切换前后方向
double moveWhileTurningDist = 3.0; // 距离较远时允许边转边走
bool useInclRateToSlow = false; // 使用倾斜速率来减速
double inclRateThre = 120.0;    // 倾斜速率阈值
double slowRate1 = 0.25;        // 减速率1
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false; // 使用倾斜度来停止
double inclThre = 45.0;     // 倾斜度阈值
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double speedFilterAlpha = 0.3;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;
double lastYawRateCmd = 0.0;
bool pathJustUpdated = false;
bool headingAligned = false;
double lastTargetSpeed = 0.0;
bool rotateLock = false;

nav_msgs::Path path;

enum FineAlignState
{
  kFineAlignNone = 0,
  kFineAlignWaitStop = 1,
  kFineAlignPos = 2,
  kFineAlignYaw = 3,
  kFineAlignDone = 4,
};

bool goalReached = false;
bool fineAlignEnabled = true;
double fineAlignMaxSpeed = 0.1;
double fineAlignPosThresh = 0.02;
double fineAlignStopSpeed = 0.02;
double fineAlignYawThresh = 0.03;
double fineAlignKpPos = 1.0;
double fineAlignKpYaw = 2.0;
double fineAlignMaxYawRate = 0.3;
int fineAlignState = kFineAlignNone;
double lastOdomTime = 0.0;
double lastOdomX = 0.0;
double lastOdomY = 0.0;
double odomSpeed = 0.0;

// MPC params (defaults)
double mpcHz = 30.0;
int mpcHorizon = 15;
int mpcIters = 10;
double mpcAlpha = 0.05;
double mpcMinStep = 0.05;
double vxMax = 1.0;
double vyMax = 1.0;
double wMax = 0.5;
double qx = 20.0;
double qy = 20.0;
double qyaw = 8.0;
double qfx = 40.0;
double qfy = 40.0;
double qfyaw = 20.0;
double rvx = 1.0;
double rvy = 1.0;
double rw = 1.0;
double pvx = 5.0;
double pvy = 5.0;
double pw = 5.0;

double goalYaw = 0.0;
double goalYawRel = 0.0;
bool hasGoalYaw = false;
double goalX = 0.0;
double goalY = 0.0;
bool hasGoalPos = false;
int lastProgressIdx = 0;

std::vector<double> pathX;
std::vector<double> pathY;
std::vector<double> pathYaw;
std::vector<double> pathS;

nav_msgs::Path execPath;
std::string odomFrame = "world";

inline double clampVal(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

inline double wrapAngle(double a)
{
  while (a > PI)
    a -= 2.0 * PI;
  while (a < -PI)
    a += 2.0 * PI;
  return a;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn)
{
  odomTime = odomIn->header.stamp.toSec();
  if (!odomIn->header.frame_id.empty())
    odomFrame = odomIn->header.frame_id;

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if (lastOdomTime > 0.0)
  {
    double dt = odomTime - lastOdomTime;
    if (dt > 1e-4)
    {
      double dx = vehicleX - lastOdomX;
      double dy = vehicleY - lastOdomY;
      odomSpeed = sqrt(dx * dx + dy * dy) / dt;
    }
  }
  lastOdomTime = odomTime;
  lastOdomX = vehicleX;
  lastOdomY = vehicleY;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop)
  {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow)
  {
    slowInitTime = odomIn->header.stamp.toSec();
  }

  if (pathInit)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = odomIn->header;
    pose.pose = odomIn->pose.pose;
    if (execPath.poses.empty())
    {
      execPath.header = odomIn->header;
      execPath.poses.push_back(pose);
    }
    else
    {
      const auto &last = execPath.poses.back().pose.position;
      double dx = pose.pose.position.x - last.x;
      double dy = pose.pose.position.y - last.y;
      if (dx * dx + dy * dy > 0.0004)
      {
        execPath.poses.push_back(pose);
      }
    }
  }
}

void pathHandler(const nav_msgs::Path::ConstPtr &pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++)
  {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
  pathJustUpdated = true;

  pathX.clear();
  pathY.clear();
  pathYaw.clear();
  pathS.clear();

  if (pathSize > 0)
  {
    pathX.reserve(pathSize);
    pathY.reserve(pathSize);
    pathYaw.reserve(pathSize);
    pathS.reserve(pathSize);
    for (int i = 0; i < pathSize; i++)
    {
      pathX.push_back(path.poses[i].pose.position.x);
      pathY.push_back(path.poses[i].pose.position.y);
    }

    goalYawRel = wrapAngle(goalYaw - vehicleYawRec);

    double accum = 0.0;
    pathS.push_back(0.0);
    for (int i = 0; i < pathSize - 1; i++)
    {
      double dx = pathX[i + 1] - pathX[i];
      double dy = pathY[i + 1] - pathY[i];
      pathYaw.push_back(atan2(dy, dx));
      accum += sqrt(dx * dx + dy * dy);
      pathS.push_back(accum);
    }

    if (pathSize >= 2)
    {
      if (hasGoalYaw)
        pathYaw.push_back(goalYawRel);
      else
        pathYaw.push_back(pathYaw.back());
    }
    else
    {
      pathYaw.push_back(goalYawRel);
    }
  }

  execPath.poses.clear();
  execPath.header.frame_id = odomFrame;
  lastProgressIdx = 0;
}

void speedHandler(const std_msgs::Float32::ConstPtr &speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0)
  {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::Int8::ConstPtr &stop)
{
  safetyStop = stop->data;
}

void goalReachedHandler(const std_msgs::Bool::ConstPtr &msg)
{
  goalReached = msg->data;
  if (!goalReached)
  {
    fineAlignState = kFineAlignNone;
  }
}

void goalHandler(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
  goalX = goal->pose.position.x;
  goalY = goal->pose.position.y;
  hasGoalPos = true;
  tf::Quaternion q(
      goal->pose.orientation.x,
      goal->pose.orientation.y,
      goal->pose.orientation.z,
      goal->pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  goalYaw = yaw;
  hasGoalYaw = true;
  if (pathInit)
  {
    goalYawRel = wrapAngle(goalYaw - vehicleYawRec);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("lookAheadDis", lookAheadDis);
  nhPrivate.getParam("lookAheadSpeedGain", lookAheadSpeedGain);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("useYawRateFilter", useYawRateFilter);
  nhPrivate.getParam("yawRateFilterAlpha", yawRateFilterAlpha);
  nhPrivate.getParam("useYawRateRateLimit", useYawRateRateLimit);
  nhPrivate.getParam("yawRateMaxDelta", yawRateMaxDelta);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("switchTimeThre", switchTimeThre);
  nhPrivate.getParam("dirDiffThre", dirDiffThre);
  nhPrivate.getParam("dirDiffGoThre", dirDiffGoThre);
  nhPrivate.getParam("dirDiffStopThre", dirDiffStopThre);
  nhPrivate.getParam("dirDiffTurnOnlyThre", dirDiffTurnOnlyThre);
  nhPrivate.getParam("turnOnlyDistThre", turnOnlyDistThre);
  nhPrivate.getParam("stopDisThre", stopDisThre);
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
  nhPrivate.getParam("posOnlyDistThre", posOnlyDistThre);
  nhPrivate.getParam("nearGoalNoSwitchDist", nearGoalNoSwitchDist);
  nhPrivate.getParam("moveWhileTurningDist", moveWhileTurningDist);
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
  nhPrivate.getParam("inclRateThre", inclRateThre);
  nhPrivate.getParam("slowRate1", slowRate1);
  nhPrivate.getParam("slowRate2", slowRate2);
  nhPrivate.getParam("slowTime1", slowTime1);
  nhPrivate.getParam("slowTime2", slowTime2);
  nhPrivate.getParam("useInclToStop", useInclToStop);
  nhPrivate.getParam("inclThre", inclThre);
  nhPrivate.getParam("stopTime", stopTime);
  nhPrivate.getParam("noRotAtStop", noRotAtStop);
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
  nhPrivate.getParam("speedFilterAlpha", speedFilterAlpha);
  nhPrivate.getParam("odom_topic", ODOM_TOPIC);  //状态估计话题
  nhPrivate.param("fine_align_enabled", fineAlignEnabled, true);
  nhPrivate.param("fine_align_max_speed", fineAlignMaxSpeed, 0.1);
  nhPrivate.param("fine_align_pos_thresh", fineAlignPosThresh, 0.02);
  nhPrivate.param("fine_align_stop_speed", fineAlignStopSpeed, 0.02);
  nhPrivate.param("fine_align_yaw_thresh", fineAlignYawThresh, 0.03);
  nhPrivate.param("fine_align_kp_pos", fineAlignKpPos, 1.0);
  nhPrivate.param("fine_align_kp_yaw", fineAlignKpYaw, 2.0);
  nhPrivate.param("fine_align_max_yaw_rate", fineAlignMaxYawRate, 0.3);
  nhPrivate.param("mpc_hz", mpcHz, 30.0);
  nhPrivate.param("mpc_horizon", mpcHorizon, 15);
  nhPrivate.param("mpc_iters", mpcIters, 10);
  nhPrivate.param("mpc_alpha", mpcAlpha, 0.05);
  nhPrivate.param("mpc_min_step", mpcMinStep, 0.05);
  nhPrivate.param("vx_max", vxMax, 1.0);
  nhPrivate.param("vy_max", vyMax, 1.0);
  nhPrivate.param("w_max", wMax, 0.5);
  nhPrivate.param("mpc_qx", qx, 20.0);
  nhPrivate.param("mpc_qy", qy, 20.0);
  nhPrivate.param("mpc_qyaw", qyaw, 8.0);
  nhPrivate.param("mpc_qfx", qfx, 40.0);
  nhPrivate.param("mpc_qfy", qfy, 40.0);
  nhPrivate.param("mpc_qfyaw", qfyaw, 20.0);
  nhPrivate.param("mpc_rvx", rvx, 1.0);
  nhPrivate.param("mpc_rvy", rvy, 1.0);
  nhPrivate.param("mpc_rw", rw, 1.0);
  nhPrivate.param("mpc_pvx", pvx, 5.0);
  nhPrivate.param("mpc_pvy", pvy, 5.0);
  nhPrivate.param("mpc_pw", pw, 5.0);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>(ODOM_TOPIC, 5, odomHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/local_path", 5, pathHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);

  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8>("/stop", 5, stopHandler);
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, goalHandler);
  ros::Subscriber subGoalReached = nh.subscribe<std_msgs::Bool>("/goal_reached", 5, goalReachedHandler);

  ros::Publisher pubSpeed_1 = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel_1", 5);
  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  ros::Publisher pubMpcPredPath = nh.advertise<nav_msgs::Path>("/mpc_pred_path", 5);
  ros::Publisher pubMpcExecPath = nh.advertise<nav_msgs::Path>("/mpc_exec_path", 5);
  geometry_msgs::Twist cmd_vel;
  
  geometry_msgs::TwistStamped cmd_vel_1;
  cmd_vel_1.header.frame_id = "body";

  if (autonomyMode)
  {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0)
      joySpeed = 0;
    else if (joySpeed > 1.0)
      joySpeed = 1.0;
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  double lastMpcTime = 0.0;
  std::vector<double> uPrev(3, 0.0);
  double mpcDt = 1.0 / std::max(1.0, mpcHz);

  while (status)
  {
    ros::spinOnce();

    if (odomTime > 0.0 && (odomTime - lastMpcTime) >= mpcDt)
    {
      lastMpcTime = odomTime;
      double vxCmd = 0.0;
      double vyCmd = 0.0;
      double wCmd = 0.0;

      nav_msgs::Path predPathMsg;
      predPathMsg.header.stamp = ros::Time().fromSec(odomTime);
      predPathMsg.header.frame_id = "body";

      if (pathInit && !pathX.empty())
      {
        double vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
        double vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) + cos(vehicleYawRec) * (vehicleY - vehicleYRec);
        double vehicleYawRel = wrapAngle(vehicleYaw - vehicleYawRec);

        int pathSize = static_cast<int>(pathX.size());
        double endDx = pathX.back() - vehicleXRel;
        double endDy = pathY.back() - vehicleYRel;
        double endDis = sqrt(endDx * endDx + endDy * endDy);
        double endYawErr = wrapAngle(goalYawRel - vehicleYawRel);
        bool doMpc = true;
        if (rotateLock || pathSize <= 1 || endDis < stopDisThre)
        {
          rotateLock = true;
          vxCmd = 0.0;
          vyCmd = 0.0;
          if (fabs(endYawErr) > dirDiffStopThre)
          {
            double w = stopYawRateGain * endYawErr;
            wCmd = clampVal(w, -wMax, wMax);
          }
          else
          {
            wCmd = 0.0;
            rotateLock = false;
          }
          uPrev[0] = 0.0;
          uPrev[1] = 0.0;
          uPrev[2] = wCmd;
          doMpc = false;
        }

        if (doMpc)
        {
          int startIdx = std::min(lastProgressIdx, pathSize - 1);
          double bestDis = 1e9;
          int bestIdx = startIdx;
          for (int i = startIdx; i < pathSize; i++)
          {
            double dx = pathX[i] - vehicleXRel;
            double dy = pathY[i] - vehicleYRel;
            double d = dx * dx + dy * dy;
            if (d < bestDis)
            {
              bestDis = d;
              bestIdx = i;
            }
          }
          lastProgressIdx = bestIdx;

          double targetSpeed = maxSpeed * joySpeed;
          double stepDist = std::max(mpcMinStep, fabs(targetSpeed) * mpcDt);
          int N = std::max(1, mpcHorizon);

          std::vector<double> xref(N, pathX.back());
          std::vector<double> yref(N, pathY.back());
          std::vector<double> yawref(N, goalYawRel);

          int idx = bestIdx;
          double startS = pathS.empty() ? 0.0 : pathS[bestIdx];
          for (int k = 0; k < N; k++)
          {
            double targetS = startS + stepDist * (k + 1);
            while (idx + 1 < pathSize && pathS[idx] < targetS)
            {
              idx++;
            }
            xref[k] = pathX[idx];
            yref[k] = pathY[idx];
            if (idx < static_cast<int>(pathYaw.size()))
              yawref[k] = pathYaw[idx];
          }

        std::vector<double> ux(N, uPrev[0]);
        std::vector<double> uy(N, uPrev[1]);
        std::vector<double> uw(N, uPrev[2]);
        std::vector<double> px(N + 1, 0.0);
        std::vector<double> py(N + 1, 0.0);
        std::vector<double> pyaw(N + 1, 0.0);
        std::vector<double> gx(N, 0.0);
        std::vector<double> gy(N, 0.0);
        std::vector<double> gw(N, 0.0);

        double dvMax = maxAccel * mpcDt;
        double dwMax = yawRateMaxDelta * PI / 180.0 * mpcDt;

        for (int iter = 0; iter < mpcIters; iter++)
        {
          px[0] = vehicleXRel;
          py[0] = vehicleYRel;
          pyaw[0] = vehicleYawRel;
          for (int k = 0; k < N; k++)
          {
            px[k + 1] = px[k] + mpcDt * ux[k];
            py[k + 1] = py[k] + mpcDt * uy[k];
            pyaw[k + 1] = wrapAngle(pyaw[k] + mpcDt * uw[k]);
          }

          double sumx = 0.0;
          double sumy = 0.0;
          double sumyaw = 0.0;
          for (int k = N - 1; k >= 0; k--)
          {
            double wqx = (k == N - 1) ? (qx + qfx) : qx;
            double wqy = (k == N - 1) ? (qy + qfy) : qy;
            double wqyaw = (k == N - 1) ? (qyaw + qfyaw) : qyaw;
            double ex = px[k + 1] - xref[k];
            double ey = py[k + 1] - yref[k];
            double eyaw = wrapAngle(pyaw[k + 1] - yawref[k]);
            sumx += 2.0 * wqx * ex;
            sumy += 2.0 * wqy * ey;
            sumyaw += 2.0 * wqyaw * eyaw;
            gx[k] = mpcDt * sumx;
            gy[k] = mpcDt * sumy;
            gw[k] = mpcDt * sumyaw;
          }

          for (int k = 0; k < N; k++)
          {
            gx[k] += 2.0 * rvx * ux[k];
            gy[k] += 2.0 * rvy * uy[k];
            gw[k] += 2.0 * rw * uw[k];

            if (k == 0)
            {
              gx[k] += 2.0 * pvx * (ux[k] - uPrev[0]);
              gy[k] += 2.0 * pvy * (uy[k] - uPrev[1]);
              gw[k] += 2.0 * pw * (uw[k] - uPrev[2]);
            }
            else if (k == N - 1)
            {
              gx[k] += 2.0 * pvx * (ux[k] - ux[k - 1]);
              gy[k] += 2.0 * pvy * (uy[k] - uy[k - 1]);
              gw[k] += 2.0 * pw * (uw[k] - uw[k - 1]);
            }
            else
            {
              gx[k] += 2.0 * pvx * (2.0 * ux[k] - ux[k - 1] - ux[k + 1]);
              gy[k] += 2.0 * pvy * (2.0 * uy[k] - uy[k - 1] - uy[k + 1]);
              gw[k] += 2.0 * pw * (2.0 * uw[k] - uw[k - 1] - uw[k + 1]);
            }
          }

          for (int k = 0; k < N; k++)
          {
            ux[k] -= mpcAlpha * gx[k];
            uy[k] -= mpcAlpha * gy[k];
            uw[k] -= mpcAlpha * gw[k];
          }

          for (int k = 0; k < N; k++)
          {
            double vxLo = -vxMax;
            double vxHi = vxMax;
            double vyLo = -vyMax;
            double vyHi = vyMax;
            double wLo = -wMax;
            double wHi = wMax;

            if (k == 0)
            {
              vxLo = std::max(vxLo, uPrev[0] - dvMax);
              vxHi = std::min(vxHi, uPrev[0] + dvMax);
              vyLo = std::max(vyLo, uPrev[1] - dvMax);
              vyHi = std::min(vyHi, uPrev[1] + dvMax);
              wLo = std::max(wLo, uPrev[2] - dwMax);
              wHi = std::min(wHi, uPrev[2] + dwMax);
            }
            else
            {
              vxLo = std::max(vxLo, ux[k - 1] - dvMax);
              vxHi = std::min(vxHi, ux[k - 1] + dvMax);
              vyLo = std::max(vyLo, uy[k - 1] - dvMax);
              vyHi = std::min(vyHi, uy[k - 1] + dvMax);
              wLo = std::max(wLo, uw[k - 1] - dwMax);
              wHi = std::min(wHi, uw[k - 1] + dwMax);
            }

            ux[k] = clampVal(ux[k], vxLo, vxHi);
            uy[k] = clampVal(uy[k], vyLo, vyHi);
            uw[k] = clampVal(uw[k], wLo, wHi);
          }
        }

        vxCmd = ux[0];
        vyCmd = uy[0];
        wCmd = uw[0];
        uPrev[0] = vxCmd;
        uPrev[1] = vyCmd;
        uPrev[2] = wCmd;

          double cosYawRec = cos(vehicleYawRec);
          double sinYawRec = sin(vehicleYawRec);
          double cosYawCur = cos(vehicleYaw);
          double sinYawCur = sin(vehicleYaw);
          predPathMsg.poses.resize(N);
          for (int k = 0; k < N; k++)
          {
            double wx = cosYawRec * px[k + 1] - sinYawRec * py[k + 1] + vehicleXRec;
            double wy = sinYawRec * px[k + 1] + cosYawRec * py[k + 1] + vehicleYRec;
            double bx = cosYawCur * (wx - vehicleX) + sinYawCur * (wy - vehicleY);
            double by = -sinYawCur * (wx - vehicleX) + cosYawCur * (wy - vehicleY);
            double wyaw = wrapAngle(pyaw[k + 1] + vehicleYawRec);
            double byaw = wrapAngle(wyaw - vehicleYaw);

            geometry_msgs::PoseStamped pose;
            pose.header = predPathMsg.header;
            pose.pose.position.x = bx;
            pose.pose.position.y = by;
            pose.pose.position.z = 0.0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(byaw);
            predPathMsg.poses[k] = pose;
          }
        }

      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0)
      {
        vxCmd = 0.0;
        vyCmd = 0.0;
        wCmd = 0.0;
      }

      bool fineAlignActive = false;
      if (fineAlignEnabled && goalReached && hasGoalPos && safetyStop >= 2)
      {
        if (fineAlignState == kFineAlignNone)
          fineAlignState = kFineAlignWaitStop;

        double dx = goalX - vehicleX;
        double dy = goalY - vehicleY;
        double dist = sqrt(dx * dx + dy * dy);
        double cosYaw = cos(vehicleYaw);
        double sinYaw = sin(vehicleYaw);
        double bx = cosYaw * dx + sinYaw * dy;
        double by = -sinYaw * dx + cosYaw * dy;

        fineAlignActive = true;
        if (fineAlignState == kFineAlignWaitStop)
        {
          vxCmd = 0.0;
          vyCmd = 0.0;
          wCmd = 0.0;
          if (odomSpeed <= fineAlignStopSpeed)
          {
            fineAlignState = kFineAlignPos;
          }
        }
        else if (fineAlignState == kFineAlignPos)
        {
          if (dist <= fineAlignPosThresh)
          {
            vxCmd = 0.0;
            vyCmd = 0.0;
            wCmd = 0.0;
            fineAlignState = kFineAlignYaw;
          }
          else
          {
            vxCmd = clampVal(fineAlignKpPos * bx, -fineAlignMaxSpeed, fineAlignMaxSpeed);
            vyCmd = clampVal(fineAlignKpPos * by, -fineAlignMaxSpeed, fineAlignMaxSpeed);
            wCmd = 0.0;
          }
        }
        else if (fineAlignState == kFineAlignYaw)
        {
          if (!hasGoalYaw)
          {
            vxCmd = 0.0;
            vyCmd = 0.0;
            wCmd = 0.0;
            fineAlignState = kFineAlignDone;
          }
          else
          {
            double yawErr = wrapAngle(goalYaw - vehicleYaw);
            if (fabs(yawErr) <= fineAlignYawThresh)
            {
              vxCmd = 0.0;
              vyCmd = 0.0;
              wCmd = 0.0;
              fineAlignState = kFineAlignDone;
            }
            else
            {
              vxCmd = 0.0;
              vyCmd = 0.0;
              wCmd = clampVal(fineAlignKpYaw * yawErr, -fineAlignMaxYawRate, fineAlignMaxYawRate);
            }
          }
        }
        else if (fineAlignState == kFineAlignDone)
        {
          vxCmd = 0.0;
          vyCmd = 0.0;
          wCmd = 0.0;
        }
      }

      if (hasGoalPos && hasGoalYaw)
      {
        double dxg = goalX - vehicleX;
        double dyg = goalY - vehicleY;
        double goalDis = sqrt(dxg * dxg + dyg * dyg);
        double goalYawErr = wrapAngle(vehicleYaw - goalYaw);
        if (goalDis < stopDisThre && fabs(goalYawErr) < dirDiffStopThre)
        {
          vxCmd = 0.0;
          vyCmd = 0.0;
          wCmd = 0.0;
          uPrev[0] = 0.0;
          uPrev[1] = 0.0;
          uPrev[2] = 0.0;
        }
      }

      if (!fineAlignActive)
      {
        if (safetyStop >= 1)
        {
          vxCmd = 0.0;
          vyCmd = 0.0;
        }
        if (safetyStop >= 2)
        {
          wCmd = 0.0;
        }
      }

      cmd_vel_1.header.stamp = ros::Time().fromSec(odomTime);
      cmd_vel_1.twist.linear.x = vxCmd;
      cmd_vel_1.twist.linear.y = vyCmd;
      cmd_vel_1.twist.angular.z = wCmd;
      pubSpeed_1.publish(cmd_vel_1);

      cmd_vel.linear.x = vxCmd;
      cmd_vel.linear.y = vyCmd;
      cmd_vel.angular.z = wCmd;
      pubSpeed.publish(cmd_vel);

      if (!predPathMsg.poses.empty())
      {
        pubMpcPredPath.publish(predPathMsg);
      }

      if (!execPath.poses.empty())
      {
        execPath.header.stamp = ros::Time().fromSec(odomTime);
        execPath.header.frame_id = odomFrame;
        pubMpcExecPath.publish(execPath);
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
