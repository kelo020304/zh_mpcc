#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>

#include <algorithm>
#include <vector>

#include "hpipm_d_ocp_qp_ipm.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "yhs_can_msgs/ctrl_cmd.h"
using std::vector;

const double PI = 3.141592653589793;

static std::string ODOM_TOPIC;

// Parameters (defaults)
double mpcHz = 30.0;
int mpcHorizon = 15;
double mpcDt = 0.0333;
double vxMax = 1.0;
double vyMax = 1.0;
double wMax = 0.5;
double vsMax = 1.0;
double vsMin = -1.0;  // Allow negative for reverse
double sTrust = 1.0;

double qC = 20.0;
double qL = 5.0;
double qYaw = 6.0;
double qC_f = 40.0;
double qL_f = 10.0;
double qYaw_f = 12.0;
double rVx = 1.0;
double rVy = 1.0;
double rW = 1.0;
double rVs = 0.5;
double qVs = 0.2; // progress reward (linear term)

// **NEW: Direction guidance parameters**
double qVsDir = 5.0;      // Weight for direction guidance (encourage vs to match path direction)
double qGearSwitch = 10.0; // Penalty for gear switching (discourage frequent direction changes)
double rInPlaceRot = 0.1;  // Reduced control penalty for in-place rotation segments

// **NEW: Acceleration constraint (control smoothing)**
bool enableMpcDuPenalty = true;  // Enable soft constraint on control changes
double rDuVx = 10.0;  // Penalty weight for vx change
double rDuVy = 10.0;  // Penalty weight for vy change
double rDuW = 5.0;    // Penalty weight for w change
double rDuVs = 5.0;   // Penalty weight for vs change

double stopDisThre = 0.2;
double maxSpeed = 1.0;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double alignDistThre = 0.5;
double alignPosThre = 0.05;
double alignPosSpeed = 0.05;
double alignYawThreDeg = 10.0;
double alignYawSpeed = 0.3;
double alignTimeoutS = 8.0;

// **MODIFIED: Simplified gear control - let optimizer decide**
bool usePathDirectionHint = true;  // Use path direction as soft guidance
double inPlaceRotThreshold = 0.05; // Threshold to detect in-place rotation segments [m]
double rotateYawThreDeg = 20.0;

// cmd smoothing
bool enableCmdSmoothing = false;
double cmdSmoothTau = 0.2;   // seconds
double cmdMaxAccelLin = 1.5; // m/s^2
double cmdMaxAccelAng = 2.0; // rad/s^2
std::string chassisType = "omni";

float joySpeed = 0;
float joySpeedRaw = 0;
double joyTime = 0;
int safetyStop = 0;
bool goalReached = false;
bool goal_reached_pub = false;

double goalYaw = 0.0;
double goalX = 0.0;
double goalY = 0.0;
bool hasGoalYaw = false;
bool hasGoalPos = false;

float vehicleX = 0;
float vehicleY = 0;
float vehicleYaw = 0;
double sensorOffsetX = 0.0;
double sensorOffsetY = 0.0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleYawRec = 0;

double odomTime = 0;
bool pathInit = false;
bool pathJustUpdated = false;

nav_msgs::Path path;
std::vector<double> pathX;
std::vector<double> pathY;
std::vector<double> pathYaw;
std::vector<double> pathS;
std::vector<int> pathDir;     // **NEW: Path direction indicator (+1=forward, -1=reverse, 0=in-place)

static geometry_msgs::Twist targetCmd;
static geometry_msgs::Twist lastCmd;
static double lastCmdTime = 0.0;
static double lastVsSign = 1.0;  // **NEW: Track previous direction for gear switch penalty

// **NEW: Track previous control for acceleration penalty**
static std::vector<double> lastU = {0.0, 0.0, 0.0, 0.0};  // [vx, vy, w, vs]
static bool hasLastU = false;

static ros::Publisher pubGoalReached;
static double align_start_time = 0.0;
static bool align_mode_active = false;
static bool align_log_pending = false;

inline double clampVal(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

inline double wrapAngle(double a)
{
  while (a > PI) a -= 2.0 * PI;
  while (a < -PI) a += 2.0 * PI;
  return a;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr &odomIn)
{
  odomTime = odomIn->header.stamp.toSec();
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
}

void speedHandler(const std_msgs::Float32::ConstPtr &speed)
{
  double speedTime = ros::Time::now().toSec();
  joySpeedRaw = speed->data;
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0)
  {
    joySpeed = speed->data / std::max(1.0, maxSpeed);
  }
  joyTime = speedTime;
}

void stopHandler(const std_msgs::Int8::ConstPtr &stop)
{
  safetyStop = stop->data;
}

void goalReachedHandler(const std_msgs::Bool::ConstPtr &msg)
{
  goalReached = msg->data;
  if (!goalReached) goal_reached_pub = false;
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
  const double qn = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
  if (!isfinite(qn) || qn < 1e-3)
  {
    hasGoalYaw = false;
    ROS_INFO("\033[36minvalid goal yaw, skip yaw align\033[0m");
  }
  else
  {
    if (fabs(qn - 1.0) > 1e-3) q = q / qn;
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    goalYaw = yaw;
    hasGoalYaw = true;
  }
  align_start_time = 0.0;
  align_mode_active = false;
}

// **NEW: Compute path direction indicators**
static void computePathYawAndDirection()
{
  const int n = static_cast<int>(pathX.size());
  pathYaw.assign(n, 0.0);
  pathS.assign(n, 0.0);
  pathDir.assign(n, 1);  // Default forward
  if (n <= 1) return;

  // Compute yaw and s from path geometry
  for (int i = 0; i < n - 1; i++)
  {
    double dx = pathX[i + 1] - pathX[i];
    double dy = pathY[i + 1] - pathY[i];
    double ds = hypot(dx, dy);
    pathYaw[i] = atan2(dy, dx);
    pathS[i + 1] = pathS[i] + ds;
  }
  pathYaw[n - 1] = pathYaw[n - 2];

  // **IMPROVED: Direction detection with look-ahead**
  for (int i = 0; i < n; i++)
  {
    // Look ahead to accumulate sufficient distance
    double accumDist = 0.0;
    double totalDx = 0.0;
    double totalDy = 0.0;
    int lookAheadIdx = i;

    while (lookAheadIdx + 1 < n && accumDist < inPlaceRotThreshold)
    {
      double dx = pathX[lookAheadIdx + 1] - pathX[lookAheadIdx];
      double dy = pathY[lookAheadIdx + 1] - pathY[lookAheadIdx];
      totalDx += dx;
      totalDy += dy;
      accumDist += hypot(dx, dy);
      lookAheadIdx++;
    }

    // Check if this is truly in-place rotation
    bool isInPlaceRotation = false;
    if (accumDist < inPlaceRotThreshold * 0.5)
    {
      double yawChange = (lookAheadIdx > i) ? fabs(wrapAngle(pathYaw[lookAheadIdx] - pathYaw[i])) : 0.0;
      if (yawChange > rotateYawThreDeg * PI / 180.0)
      {
        isInPlaceRotation = true;
      }
    }

    if (isInPlaceRotation)
    {
      pathDir[i] = 0;  // In-place rotation
    }
    else if (accumDist > 1e-4)
    {
      // Sufficient displacement - use motion direction as reference
      double motionYaw = atan2(totalDy, totalDx);
      pathDir[i] = 1;  // Default forward (no orientation info in this mode)
    }
    else
    {
      pathDir[i] = (i > 0) ? pathDir[i - 1] : 1;
    }
  }

  // unwrap yaw for smooth interpolation
  for (int i = 1; i < n; i++)
  {
    double dyaw = wrapAngle(pathYaw[i] - pathYaw[i - 1]);
    pathYaw[i] = pathYaw[i - 1] + dyaw;
  }
}

void pathHandler(const nav_msgs::Path::ConstPtr &pathIn)
{
  int pathSize = pathIn->poses.size();
  if (pathSize <= 0) return;
  path.poses.resize(pathSize);
  pathX.resize(pathSize);
  pathY.resize(pathSize);
  pathYaw.resize(pathSize);
  pathS.resize(pathSize);
  pathDir.resize(pathSize);

  bool anyYaw = false;
  for (int i = 0; i < pathSize; i++)
  {
    pathX[i] = pathIn->poses[i].pose.position.x;
    pathY[i] = pathIn->poses[i].pose.position.y;

    const auto &q = pathIn->poses[i].pose.orientation;
    double yaw = tf::getYaw(q);
    pathYaw[i] = yaw;
    if (std::fabs(yaw) > 1e-3 || std::fabs(q.x) > 1e-6 || std::fabs(q.y) > 1e-6 || std::fabs(q.z) > 1e-6 || std::fabs(q.w - 1.0) > 1e-6)
    {
      anyYaw = true;
    }
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleYawRec = vehicleYaw;

  const bool usePathOrientation = (chassisType == "omni" || chassisType == "holonomic");
  if (usePathOrientation && anyYaw)
  {
    // Use path-provided orientations
    pathS[0] = 0.0;
    for (int i = 1; i < pathSize; i++)
    {
      double dx = pathX[i] - pathX[i - 1];
      double dy = pathY[i] - pathY[i - 1];
      double ds = hypot(dx, dy);
      pathS[i] = pathS[i - 1] + ds;
    }

    // **IMPROVED: Infer direction with look-ahead strategy**
    // For dense paths, accumulate distance until threshold is reached
    for (int i = 0; i < pathSize; i++)
    {
      // Look ahead to accumulate sufficient distance
      double accumDist = 0.0;
      double totalDx = 0.0;
      double totalDy = 0.0;
      int lookAheadIdx = i;

      // Accumulate distance by looking ahead
      while (lookAheadIdx + 1 < pathSize && accumDist < inPlaceRotThreshold)
      {
        double dx = pathX[lookAheadIdx + 1] - pathX[lookAheadIdx];
        double dy = pathY[lookAheadIdx + 1] - pathY[lookAheadIdx];
        totalDx += dx;
        totalDy += dy;
        accumDist += hypot(dx, dy);
        lookAheadIdx++;
      }

      // Check if this is truly in-place rotation
      bool isInPlaceRotation = false;
      if (accumDist < inPlaceRotThreshold * 0.5)
      {
        // Very small total displacement - check yaw change
        double yawChange = (lookAheadIdx > i) ? fabs(wrapAngle(pathYaw[lookAheadIdx] - pathYaw[i])) : 0.0;
        if (yawChange > rotateYawThreDeg * PI / 180.0)
        {
          isInPlaceRotation = true;
        }
      }

      if (isInPlaceRotation)
      {
        pathDir[i] = 0;  // In-place rotation
      }
      else if (accumDist > 1e-4)
      {
        // Sufficient displacement - determine direction
        double motionYaw = atan2(totalDy, totalDx);
        double pathYawLocal = pathYaw[i];
        double headingDiff = wrapAngle(pathYawLocal - motionYaw);

        // If path yaw points backward relative to motion, this is a reverse segment
        if (fabs(headingDiff) > PI / 2)
        {
          pathDir[i] = -1;  // Reverse
        }
        else
        {
          pathDir[i] = 1;   // Forward
        }
      }
      else
      {
        // No displacement, inherit from previous or default to forward
        pathDir[i] = (i > 0) ? pathDir[i - 1] : 1;
      }
    }

    // unwrap yaw for smooth interpolation
    for (int i = 1; i < pathSize; i++)
    {
      double dyaw = wrapAngle(pathYaw[i] - pathYaw[i - 1]);
      pathYaw[i] = pathYaw[i - 1] + dyaw;
    }
  }
  else
  {
    computePathYawAndDirection();
  }
  pathInit = true;
  pathJustUpdated = true;

  // Debug output - show overall path statistics
  int forwardCount = 0, reverseCount = 0, rotateCount = 0;
  for (int i = 0; i < pathSize; i++)
  {
    if (pathDir[i] == 1) forwardCount++;
    else if (pathDir[i] == -1) reverseCount++;
    else if (pathDir[i] == 0) rotateCount++;
  }

  double pathLength = pathS.empty() ? 0.0 : pathS.back();
  ROS_INFO("\033[36mPath updated: %d points, length=%.2fm, dirs: fwd=%d rev=%d rot=%d\033[0m",
           pathSize, pathLength, forwardCount, reverseCount, rotateCount);

  // Show first few points with detailed debug
  for (int i = 0; i < std::min(1, pathSize); i++)
  {
    const char* dirStr = (pathDir[i] == 1) ? "fwd" : (pathDir[i] == -1) ? "rev" : "rot";
    ROS_INFO("  dir=%s yaw=%.2f pos=(%.2f,%.2f)", i, dirStr, pathYaw[i], pathX[i], pathY[i]);
  }

  // Additional debug: show raw quaternion of first point
  if (pathSize > 0)
  {
    const auto &q = pathIn->poses[0].pose.orientation;
    ROS_INFO("  First point raw quat: x=%.3f y=%.3f z=%.3f w=%.3f (yaw=%.3f)",
             q.x, q.y, q.z, q.w, tf::getYaw(q));
  }
}

static void samplePathByS(double s, double &x, double &y, double &yaw, int &dir)
{
  const int n = static_cast<int>(pathS.size());
  if (n <= 1)
  {
    x = pathX.empty() ? 0.0 : pathX[0];
    y = pathY.empty() ? 0.0 : pathY[0];
    yaw = pathYaw.empty() ? 0.0 : pathYaw[0];
    dir = pathDir.empty() ? 1 : pathDir[0];
    return;
  }
  if (s <= pathS.front())
  {
    x = pathX.front();
    y = pathY.front();
    yaw = pathYaw.front();
    dir = pathDir.front();
    return;
  }
  if (s >= pathS.back())
  {
    x = pathX.back();
    y = pathY.back();
    yaw = pathYaw.back();
    dir = pathDir.back();
    return;
  }
  int idx = 0;
  while (idx + 1 < n && pathS[idx + 1] < s) idx++;
  const double s0 = pathS[idx];
  const double s1 = pathS[idx + 1];
  const double t = (s - s0) / std::max(1e-6, s1 - s0);
  x = pathX[idx] * (1.0 - t) + pathX[idx + 1] * t;
  y = pathY[idx] * (1.0 - t) + pathY[idx + 1] * t;

  // **FIX: Proper angle interpolation to handle ±π wraparound**
  double yaw0 = pathYaw[idx];
  double yaw1 = pathYaw[idx + 1];
  double dyaw = wrapAngle(yaw1 - yaw0);  // Shortest angular difference
  yaw = wrapAngle(yaw0 + t * dyaw);

  dir = pathDir[idx];  // Use discrete direction of nearest segment
}

struct QPSolution
{
  std::vector<double> xk; // size NX*(N+1)
  std::vector<double> uk; // size NU*N
};

// **MODIFIED: Unified MPCC solver with direction guidance**
static bool solveMpccQp(
    const std::vector<double> &x0,
    const std::vector<double> &s_guess,
    int N,
    double dt,
    double prevVsSign,
    QPSolution &sol)
{
  const int NX = 4;  // [x, y, yaw, s]
  const int NU = 4;  // [vx, vy, w, vs]

  std::vector<int> nx(N + 1, NX);
  std::vector<int> nu(N + 1, NU);
  nu[N] = 0;

  std::vector<int> nbx(N + 1, 0);
  std::vector<int> nbu(N + 1, NU);
  nbu[N] = 0;
  std::vector<int> ng(N + 1, 0);
  std::vector<int> ns(N + 1, 0);

  // allocate matrices per stage
  std::vector<std::vector<double>> A(N), B(N), b(N);
  std::vector<std::vector<double>> Q(N + 1), R(N + 1), S(N + 1), q(N + 1), r(N + 1);
  std::vector<std::vector<int>> idxbx(N + 1), idxbu(N + 1);
  std::vector<std::vector<double>> lbx(N + 1), ubx(N + 1), lbu(N + 1), ubu(N + 1);

  for (int k = 0; k < N; k++)
  {
    A[k].assign(NX * NX, 0.0);
    B[k].assign(NX * NU, 0.0);
    b[k].assign(NX, 0.0);
    // A = I
    for (int i = 0; i < NX; i++)
      A[k][i + i * NX] = 1.0;
    // B = dt * I
    for (int i = 0; i < NX; i++)
      B[k][i + i * NX] = dt;
  }

  for (int k = 0; k <= N; k++)
  {
    Q[k].assign(NX * NX, 0.0);
    q[k].assign(NX, 0.0);
    S[k].assign(NX * NU, 0.0);
    R[k].assign(NU * NU, 0.0);
    r[k].assign(NU, 0.0);
  }

  // **NEW: Track cumulative vs for gear switch penalty**
  double cumulativeVs = 0.0;

  for (int k = 0; k <= N; k++)
  {
    double x_ref = 0.0, y_ref = 0.0, yaw_ref = 0.0;
    int dir_ref = 1;
    samplePathByS(s_guess[k], x_ref, y_ref, yaw_ref, dir_ref);

    // Wrap yaw_ref to [-pi, pi] to avoid accumulation issues
    yaw_ref = wrapAngle(yaw_ref);

    const double c = cos(yaw_ref);
    const double s = sin(yaw_ref);

    const double a_cx = -s;
    const double a_cy = c;
    const double b_c = s * x_ref - c * y_ref;

    const double a_lx = c;
    const double a_ly = s;
    const double b_l = -c * x_ref - s * y_ref;

    const double wC = (k == N) ? qC_f : qC;
    const double wL = (k == N) ? qL_f : qL;
    const double wYaw = (k == N) ? qYaw_f : qYaw;

    // **MODIFIED: For in-place rotation segments, heavily penalize lag error**
    const double wC_eff = (dir_ref == 0) ? wC * 10.0 : wC;  // Increase contour weight for rotation
    const double wL_eff = (dir_ref == 0) ? wL * 0.1 : wL;   // Decrease lag weight for rotation

    // Q for x,y from contour/lag errors
    const double qxx = 2.0 * (wC_eff * a_cx * a_cx + wL_eff * a_lx * a_lx);
    const double qxy = 2.0 * (wC_eff * a_cx * a_cy + wL_eff * a_lx * a_ly);
    const double qyy = 2.0 * (wC_eff * a_cy * a_cy + wL_eff * a_ly * a_ly);

    Q[k][0 + 0 * NX] += qxx;
    Q[k][0 + 1 * NX] += qxy;
    Q[k][1 + 0 * NX] += qxy;
    Q[k][1 + 1 * NX] += qyy;

    q[k][0] += 2.0 * (wC_eff * b_c * a_cx + wL_eff * b_l * a_lx);
    q[k][1] += 2.0 * (wC_eff * b_c * a_cy + wL_eff * b_l * a_ly);

    // yaw tracking
    Q[k][2 + 2 * NX] += 2.0 * wYaw;
    q[k][2] += 2.0 * wYaw * (-yaw_ref);

    if (k < N)
    {
      // **MODIFIED: Reduce control penalty for in-place rotation**
      const double rVxEff = (dir_ref == 0) ? rVx * rInPlaceRot : rVx;
      const double rVyEff = (dir_ref == 0) ? rVy * rInPlaceRot : rVy;
      const double rVsEff = (dir_ref == 0) ? rVs * rInPlaceRot : rVs;

      R[k][0 + 0 * NU] += 2.0 * rVxEff;
      R[k][1 + 1 * NU] += 2.0 * rVyEff;
      R[k][2 + 2 * NU] += 2.0 * rW;
      R[k][3 + 3 * NU] += 2.0 * rVsEff;

      // **NEW: Acceleration penalty (control smoothing) - penalize u - u_prev**
      // Cost += rDu * (u[k] - u_prev)^2 = u[k]^T * rDu * u[k] - 2 * u_prev^T * rDu * u[k] + const
      // Add rDu to R diagonal, add -2*rDu*u_prev to r
      if (enableMpcDuPenalty && hasLastU)
      {
        R[k][0 + 0 * NU] += 2.0 * rDuVx;
        R[k][1 + 1 * NU] += 2.0 * rDuVy;
        R[k][2 + 2 * NU] += 2.0 * rDuW;
        R[k][3 + 3 * NU] += 2.0 * rDuVs;

        r[k][0] += -2.0 * rDuVx * lastU[0];
        r[k][1] += -2.0 * rDuVy * lastU[1];
        r[k][2] += -2.0 * rDuW * lastU[2];
        r[k][3] += -2.0 * rDuVs * lastU[3];
      }

      // **NEW: Direction guidance - encourage vs to match path direction**
      if (usePathDirectionHint && dir_ref != 0)
      {
        // Add linear term to encourage vs * dir_ref > 0
        // Cost = -qVsDir * dir_ref * vs = vs * (-qVsDir * dir_ref)
        r[k][3] += -qVsDir * static_cast<double>(dir_ref);
      }
      else if (dir_ref == 0)
      {
        // For in-place rotation, discourage linear motion
        r[k][3] += -qVs * 0.1;  // Small progress reward
      }
      else
      {
        // No direction hint, use standard progress reward
        r[k][3] += -qVs;
      }

      // **NEW: Gear switch penalty - penalize sign change of vs**
      // Approximate with quadratic: vs^2 if sign(vs) != prevVsSign
      // This is a simplified heuristic; proper implementation needs MIQP
      if (k == 0)
      {
        double expectedSign = (dir_ref != 0) ? static_cast<double>(dir_ref) : prevVsSign;
        if (expectedSign * prevVsSign < 0)
        {
          // Expecting to switch gears - add penalty
          R[k][3 + 3 * NU] += 2.0 * qGearSwitch;
        }
      }
    }
  }

  // bounds: fix x0 at stage 0
  nbx[0] = NX;
  idxbx[0].resize(NX);
  lbx[0].resize(NX);
  ubx[0].resize(NX);
  for (int i = 0; i < NX; i++)
  {
    idxbx[0][i] = i;
    lbx[0][i] = x0[i];
    ubx[0][i] = x0[i];
  }

  // **NEW: Add s bounds for all stages to prevent s from going out of path range**
  double sMax = pathS.empty() ? 10.0 : pathS.back();
  double sMin = 0.0;
  for (int k = 1; k <= N; k++)
  {
    nbx[k] = 1;  // Only constrain s (index 3)
    idxbx[k].resize(1);
    lbx[k].resize(1);
    ubx[k].resize(1);
    idxbx[k][0] = 3;  // s is the 4th state (index 3)
    lbx[k][0] = sMin;
    ubx[k][0] = sMax;
  }

  // **MODIFIED: Allow full bidirectional vs range**
  for (int k = 0; k < N; k++)
  {
    idxbu[k] = {0, 1, 2, 3};
    lbu[k] = {-vxMax, -vyMax, -wMax, vsMin};  // vsMin can be negative
    ubu[k] = {vxMax, vyMax, wMax, vsMax};
  }

  // set hpipm pointers
  std::vector<double*> hA(N + 1, nullptr), hB(N + 1, nullptr), hb(N + 1, nullptr);
  std::vector<double*> hQ(N + 1, nullptr), hS(N + 1, nullptr), hR(N + 1, nullptr), hq(N + 1, nullptr), hr(N + 1, nullptr);
  std::vector<int*> hidxbx(N + 1, nullptr), hidxbu(N + 1, nullptr);
  std::vector<double*> hlbx(N + 1, nullptr), hubx(N + 1, nullptr), hlbu(N + 1, nullptr), hubu(N + 1, nullptr);
  std::vector<double*> hC(N + 1, nullptr), hD(N + 1, nullptr), hlg(N + 1, nullptr), hug(N + 1, nullptr);
  std::vector<double*> hZl(N + 1, nullptr), hZu(N + 1, nullptr), hzl(N + 1, nullptr), hzu(N + 1, nullptr);
  std::vector<int*> hidxs(N + 1, nullptr), hidxs_rev(N + 1, nullptr);
  std::vector<double*> hlls(N + 1, nullptr), hlus(N + 1, nullptr);

  for (int k = 0; k < N; k++)
  {
    hA[k] = A[k].data();
    hB[k] = B[k].data();
    hb[k] = b[k].data();
  }
  for (int k = 0; k <= N; k++)
  {
    hQ[k] = Q[k].data();
    hS[k] = (k < N) ? S[k].data() : nullptr;
    hR[k] = (k < N) ? R[k].data() : nullptr;
    hq[k] = q[k].data();
    hr[k] = (k < N) ? r[k].data() : nullptr;

    if (!idxbx[k].empty())
    {
      hidxbx[k] = idxbx[k].data();
      hlbx[k] = lbx[k].data();
      hubx[k] = ubx[k].data();
    }
    if (!idxbu[k].empty())
    {
      hidxbu[k] = idxbu[k].data();
      hlbu[k] = lbu[k].data();
      hubu[k] = ubu[k].data();
    }
  }

  // ocp qp dim
  int dim_size = d_ocp_qp_dim_memsize(N);
  void *dim_mem = malloc(dim_size);
  struct d_ocp_qp_dim dim;
  d_ocp_qp_dim_create(N, &dim, dim_mem);
  d_ocp_qp_dim_set_all(nx.data(), nu.data(), nbx.data(), nbu.data(), ng.data(), ns.data(), &dim);

  // ocp qp
  int qp_size = d_ocp_qp_memsize(&dim);
  void *qp_mem = malloc(qp_size);
  struct d_ocp_qp qp;
  d_ocp_qp_create(&dim, &qp, qp_mem);
  d_ocp_qp_set_all(hA.data(), hB.data(), hb.data(),
                   hQ.data(), hS.data(), hR.data(), hq.data(), hr.data(),
                   hidxbx.data(), hlbx.data(), hubx.data(),
                   hidxbu.data(), hlbu.data(), hubu.data(),
                   NULL, NULL, NULL, NULL,  // hC, hD, hlg, hug (no general constraints)
                   NULL, NULL, NULL, NULL,  // hZl, hZu, hzl, hzu (no soft constraints)
                   NULL, NULL, NULL, NULL,  // hidxs, hidxs_rev, hlls, hlus (no soft constraints)
                   &qp);

  // ocp qp sol
  int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
  void *qp_sol_mem = malloc(qp_sol_size);
  struct d_ocp_qp_sol qp_sol;
  d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

  // ipm arg
  int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
  void *ipm_arg_mem = malloc(ipm_arg_size);
  struct d_ocp_qp_ipm_arg arg;
  d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
  enum hpipm_mode mode = SPEED;
  d_ocp_qp_ipm_arg_set_default(mode, &arg);

  // ipm
  int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
  void *ipm_mem = malloc(ipm_size);
  struct d_ocp_qp_ipm_ws workspace;
  d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

  d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);

  // extract solution
  sol.xk.assign(NX * (N + 1), 0.0);
  sol.uk.assign(NU * N, 0.0);
  vector<double> xbuf(NX, 0.0);
  vector<double> ubuf(NU, 0.0);
  for (int k = 0; k <= N; k++)
  {
    d_ocp_qp_sol_get_x(k, &qp_sol, xbuf.data());
    for (int i = 0; i < NX; i++)
      sol.xk[k * NX + i] = xbuf[i];
  }
  for (int k = 0; k < N; k++)
  {
    d_ocp_qp_sol_get_u(k, &qp_sol, ubuf.data());
    for (int i = 0; i < NU; i++)
      sol.uk[k * NU + i] = ubuf[i];
  }

  free(ipm_mem);
  free(ipm_arg_mem);
  free(qp_sol_mem);
  free(qp_mem);
  free(dim_mem);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpccfollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");

  nhPrivate.param("odom_topic", ODOM_TOPIC, std::string("/localization"));
  nhPrivate.param("sensorOffsetX", sensorOffsetX, 0.0);
  nhPrivate.param("sensorOffsetY", sensorOffsetY, 0.0);
  nhPrivate.param("mpc_hz", mpcHz, 30.0);
  nhPrivate.param("mpc_horizon", mpcHorizon, 15);
  nhPrivate.param("maxSpeed", maxSpeed, 1.0);
  nhPrivate.param("autonomyMode", autonomyMode, false);
  nhPrivate.param("autonomySpeed", autonomySpeed, 1.0);
  nhPrivate.param("joyToSpeedDelay", joyToSpeedDelay, 2.0);
  nhPrivate.param("enable_cmd_smoothing", enableCmdSmoothing, enableCmdSmoothing);
  nhPrivate.param("cmd_smooth_tau", cmdSmoothTau, cmdSmoothTau);
  nhPrivate.param("cmd_max_accel_lin", cmdMaxAccelLin, cmdMaxAccelLin);
  nhPrivate.param("cmd_max_accel_ang", cmdMaxAccelAng, cmdMaxAccelAng);
  nhPrivate.param("chassis_type", chassisType, chassisType);
  nhPrivate.param("vx_max", vxMax, 1.0);
  nhPrivate.param("vy_max", vyMax, 1.0);
  nhPrivate.param("w_max", wMax, 0.5);
  nhPrivate.param("vs_max", vsMax, 1.0);
  nhPrivate.param("vs_min", vsMin, -1.0);
  nhPrivate.param("s_trust", sTrust, 1.0);
  nhPrivate.param("qC", qC, 20.0);
  nhPrivate.param("qL", qL, 5.0);
  nhPrivate.param("qYaw", qYaw, 6.0);
  nhPrivate.param("qC_f", qC_f, 40.0);
  nhPrivate.param("qL_f", qL_f, 10.0);
  nhPrivate.param("qYaw_f", qYaw_f, 12.0);
  nhPrivate.param("rVx", rVx, 1.0);
  nhPrivate.param("rVy", rVy, 1.0);
  nhPrivate.param("rW", rW, 1.0);
  nhPrivate.param("rVs", rVs, 0.5);
  nhPrivate.param("qVs", qVs, 0.2);
  nhPrivate.param("qVsDir", qVsDir, 5.0);
  nhPrivate.param("qGearSwitch", qGearSwitch, 10.0);
  nhPrivate.param("rInPlaceRot", rInPlaceRot, 0.1);
  nhPrivate.param("enable_mpc_du_penalty", enableMpcDuPenalty, true);
  nhPrivate.param("rDuVx", rDuVx, 10.0);
  nhPrivate.param("rDuVy", rDuVy, 10.0);
  nhPrivate.param("rDuW", rDuW, 5.0);
  nhPrivate.param("rDuVs", rDuVs, 5.0);
  nhPrivate.param("stop_dis_thre", stopDisThre, 0.2);
  nhPrivate.param("align_dist_thre", alignDistThre, alignDistThre);
  nhPrivate.param("align_pos_thre", alignPosThre, alignPosThre);
  nhPrivate.param("align_pos_speed", alignPosSpeed, alignPosSpeed);
  nhPrivate.param("align_yaw_thre_deg", alignYawThreDeg, alignYawThreDeg);
  nhPrivate.param("align_yaw_speed", alignYawSpeed, alignYawSpeed);
  nhPrivate.param("align_timeout_s", alignTimeoutS, alignTimeoutS);
  nhPrivate.param("use_path_direction_hint", usePathDirectionHint, usePathDirectionHint);
  nhPrivate.param("in_place_rot_threshold", inPlaceRotThreshold, inPlaceRotThreshold);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>(ODOM_TOPIC, 5, odomHandler);
  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/local_path", 5, pathHandler);
  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32>("/speed", 5, speedHandler);
  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8>("/stop", 5, stopHandler);
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, goalHandler);
  ros::Subscriber subGoalReached = nh.subscribe<std_msgs::Bool>("/goal_reached", 5, goalReachedHandler);

  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  ros::Publisher yhs_ctrl_pub = nh.advertise<yhs_can_msgs::ctrl_cmd>("/ctrl_cmd", 1);
  pubGoalReached = nh.advertise<std_msgs::Bool>("/goal_reached", 1, true);

  ros::Publisher pubPred = nh.advertise<nav_msgs::Path>("/mpcc_pred_path", 5);

  geometry_msgs::Twist cmd_vel;

  vector<double> last_s_guess;
  ros::Rate rate(100);
  double lastMpcTime = 0.0;

  while (ros::ok())
  {
    ros::spinOnce();
    mpcDt = 1.0 / std::max(1.0, mpcHz);

    if (odomTime > 0.0 && (odomTime - lastMpcTime) >= mpcDt)
    {
      lastMpcTime = odomTime;
      double vxCmd = 0.0;
      double vyCmd = 0.0;
      double wCmd = 0.0;
      bool alignActive = false;

      if (hasGoalPos && hasGoalYaw)
      {
        double dxg = goalX - vehicleX;
        double dyg = goalY - vehicleY;
        double goalDis = sqrt(dxg * dxg + dyg * dyg);
        if (goalDis <= alignDistThre)
        {
          alignActive = true;
          if (!align_mode_active)
          {
            align_mode_active = true;
            align_start_time = odomTime;
            align_log_pending = true;
          }
          const double yawErr = wrapAngle(goalYaw - vehicleYaw);
          const double yawErrDeg = fabs(yawErr) * 180.0 / PI;
          const double goalYawDeg = goalYaw * 180.0 / PI;
          const double vehicleYawDeg = vehicleYaw * 180.0 / PI;
          auto log_align_status = [&](const char *tag) {
            ROS_INFO("\033[36malign status%s: goalDis=%.3f yawErrDeg=%.1f vx=%.3f vy=%.3f w=%.3f "
                     "goalYawDeg=%.1f vehicleYawDeg=%.1f safetyStop=%d\033[0m",
                     tag, goalDis, yawErrDeg, vxCmd, vyCmd, wCmd, goalYawDeg, vehicleYawDeg, safetyStop);
          };

          const bool alignTimeout =
              (alignTimeoutS > 0.0) && ((odomTime - align_start_time) >= alignTimeoutS);
          if (alignTimeout)
          {
            vxCmd = 0.0;
            vyCmd = 0.0;
            wCmd = 0.0;
            log_align_status(" (final)");
            if (!goal_reached_pub)
            {
              std_msgs::Bool msg;
              msg.data = true;
              pubGoalReached.publish(msg);
              goal_reached_pub = true;
              goalReached = true;
              ROS_INFO("\033[36mall done (timeout)\033[0m");
            }
          }
          else if (goalDis > alignPosThre)
          {
            // Calculate heading to goal in world frame
            double goalHeading = atan2(dyg, dxg);
            double headingBody = wrapAngle(goalHeading - vehicleYaw);

            // Smart forward/reverse decision:
            // If goal is behind us (|headingBody| > 90°) AND final yaw is similar to current yaw,
            // then reverse to goal instead of turning around
            bool shouldReverse = false;
            if (fabs(headingBody) > PI / 2.0)  // Goal is behind (> 90°)
            {
              // Check if final yaw is close to current yaw (within ±90°)
              // If yes, better to reverse; if no, better to turn and drive forward
              if (fabs(yawErr) < PI / 2.0)
              {
                shouldReverse = true;
              }
            }

            if (shouldReverse)
            {
              // Reverse to goal: flip heading by 180° and use negative speed
              headingBody = wrapAngle(headingBody + PI);
              vxCmd = -alignPosSpeed * cos(headingBody);
              vyCmd = -alignPosSpeed * sin(headingBody);
            }
            else
            {
              // Forward to goal
              vxCmd = alignPosSpeed * cos(headingBody);
              vyCmd = alignPosSpeed * sin(headingBody);
            }

            // Rotate towards final yaw simultaneously
            wCmd = (yawErr > 0.0) ? alignYawSpeed : -alignYawSpeed;
          }
          else
          {
            if (yawErrDeg <= alignYawThreDeg)
            {
              vxCmd = 0.0;
              vyCmd = 0.0;
              wCmd = 0.0;
              log_align_status(" (final)");
              if (!goal_reached_pub)
              {
                std_msgs::Bool msg;
                msg.data = true;
                pubGoalReached.publish(msg);
                goal_reached_pub = true;
                goalReached = true;
                ROS_INFO("\033[36mall done\033[0m");
              }
            }
            else
            {
              vxCmd = 0.0;
              vyCmd = 0.0;
              wCmd = (yawErr > 0.0) ? alignYawSpeed : -alignYawSpeed;
            }
          }
          if (align_log_pending)
          {
            log_align_status(" (start)");
            align_log_pending = false;
          }
        }
      }
      if (!alignActive)
      {
        align_mode_active = false;
        align_start_time = 0.0;
        align_log_pending = false;
      }

      if (!alignActive && pathInit && !pathX.empty())
      {
        double vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
        double vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) + cos(vehicleYawRec) * (vehicleY - vehicleYRec);
        double vehicleYawRel = wrapAngle(vehicleYaw - vehicleYawRec);

        int pathSize = static_cast<int>(pathX.size());
        double endDx = pathX.back() - vehicleXRel;
        double endDy = pathY.back() - vehicleYRel;
        double endDis = sqrt(endDx * endDx + endDy * endDy);
        ROS_INFO_THROTTLE(1.0, "\033[36mmpcc debug: pathSize=%d endDis=%.3f safetyStop=%d\033[0m",
                          pathSize, endDis, safetyStop);
        if (pathSize <= 1 || endDis < stopDisThre)
        {
          vxCmd = 0.0;
          vyCmd = 0.0;
          wCmd = 0.0;
        }
        else
        {
          // **IMPROVED: Find nearest point on path with precise projection**
          int bestIdx = 0;
          double bestDis = 1e9;
          double s0 = 0.0;

          for (int i = 0; i < pathSize; i++)
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

          // Refine s0 by projecting onto the segment near bestIdx
          if (pathSize >= 2 && !pathS.empty())
          {
            // Try projecting onto segment [bestIdx-1, bestIdx] and [bestIdx, bestIdx+1]
            double best_s = pathS[bestIdx];
            double best_proj_dist = bestDis;

            for (int seg = std::max(0, bestIdx - 1); seg < std::min(pathSize - 1, bestIdx + 1); seg++)
            {
              double ax = pathX[seg], ay = pathY[seg];
              double bx = pathX[seg + 1], by = pathY[seg + 1];
              double abx = bx - ax, aby = by - ay;
              double apx = vehicleXRel - ax, apy = vehicleYRel - ay;
              double ab_len2 = abx * abx + aby * aby;

              if (ab_len2 > 1e-9)
              {
                double t = (apx * abx + apy * aby) / ab_len2;
                t = std::max(0.0, std::min(1.0, t));  // Clamp to segment
                double proj_x = ax + t * abx;
                double proj_y = ay + t * aby;
                double proj_dist = (vehicleXRel - proj_x) * (vehicleXRel - proj_x) +
                                   (vehicleYRel - proj_y) * (vehicleYRel - proj_y);

                if (proj_dist < best_proj_dist)
                {
                  best_proj_dist = proj_dist;
                  best_s = pathS[seg] + t * (pathS[seg + 1] - pathS[seg]);
                }
              }
            }
            s0 = best_s;
          }
          else
          {
            s0 = pathS.empty() ? 0.0 : pathS[bestIdx];
          }

          // **MODIFIED: Remove manual gear selection, let optimizer decide**
          int N = std::max(1, mpcHorizon);
          vector<double> s_guess(N + 1, s0);

          // Simple initialization: assume constant progress
          double step_s = 0.1;  // Small default step
          for (int k = 1; k <= N; k++)
            s_guess[k] = s_guess[k - 1] + step_s;

          if (!last_s_guess.empty() && static_cast<int>(last_s_guess.size()) == N + 1 && !pathJustUpdated)
          {
            // Use previous solution as warm start
            s_guess = last_s_guess;
            s_guess[0] = s0;
            // Ensure monotonicity (can go forward or backward)
            for (int k = 1; k <= N; k++)
            {
              // No hard constraint, let optimizer decide
            }
          }
          pathJustUpdated = false;

          vector<double> x0 = {vehicleXRel, vehicleYRel, vehicleYawRel, s0};
          QPSolution sol;
          if (solveMpccQp(x0, s_guess, N, mpcDt, lastVsSign, sol))
          {
            double speedScale = std::max(0.0f, std::min(1.0f, joySpeed));
            if (autonomyMode && joySpeedRaw == 0)
            {
              speedScale = std::max(0.0, std::min(1.0, autonomySpeed / std::max(1.0, maxSpeed)));
            }
            // MPCC outputs velocities in PATH frame (relative to path start)
            // Need to transform: path frame -> world frame -> body frame

            // Step 1: Path frame -> World frame
            double vx_path = sol.uk[0] * speedScale;
            double vy_path = sol.uk[1] * speedScale;
            double c_rec = cos(vehicleYawRec);
            double s_rec = sin(vehicleYawRec);
            double vx_world = c_rec * vx_path - s_rec * vy_path;
            double vy_world = s_rec * vx_path + c_rec * vy_path;

            // Step 2: World frame -> Body frame
            double c_veh = cos(vehicleYaw);
            double s_veh = sin(vehicleYaw);
            vxCmd = c_veh * vx_world + s_veh * vy_world;   // body forward
            vyCmd = -s_veh * vx_world + c_veh * vy_world;  // body lateral

            wCmd = sol.uk[2] * speedScale;
            double vs = sol.uk[3];

            // **NEW: Update last control for acceleration penalty**
            if (enableMpcDuPenalty)
            {
              lastU[0] = sol.uk[0];  // Store unscaled control
              lastU[1] = sol.uk[1];
              lastU[2] = sol.uk[2];
              lastU[3] = sol.uk[3];
              hasLastU = true;
            }

            // **NEW: Update last direction for gear switch penalty**
            if (fabs(vs) > 0.01)
            {
              lastVsSign = (vs > 0) ? 1.0 : -1.0;
            }

            ROS_INFO_THROTTLE(0.5, "\033[36mmpcc:\033[0m");
            ROS_INFO_THROTTLE(0.5, "  path: vx=%.3f vy=%.3f", vx_path, vy_path);
            ROS_INFO_THROTTLE(0.5, "  world: vx=%.3f vy=%.3f", vx_world, vy_world);
            ROS_INFO_THROTTLE(0.5, "  body: vx=%.3f vy=%.3f w=%.3f", vxCmd, vyCmd, wCmd);
            ROS_INFO_THROTTLE(0.5, "  vs=%.3f dir=%s yawRec=%.1f° yaw=%.1f°",
                              vs, (vs >= 0) ? "fwd" : "rev",
                              vehicleYawRec * 180.0 / PI, vehicleYaw * 180.0 / PI);
            last_s_guess.resize(N + 1);
            for (int k = 0; k <= N; k++)
              last_s_guess[k] = sol.xk[k * 4 + 3];

            nav_msgs::Path predPathMsg;
            predPathMsg.header.stamp = ros::Time().fromSec(odomTime);
            predPathMsg.header.frame_id = "body";
            predPathMsg.poses.resize(N);
            for (int k = 0; k < N; k++)
            {
              geometry_msgs::PoseStamped pose;
              pose.header = predPathMsg.header;
              pose.pose.position.x = sol.xk[(k + 1) * 4 + 0];
              pose.pose.position.y = sol.xk[(k + 1) * 4 + 1];
              pose.pose.position.z = 0.0;
              pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol.xk[(k + 1) * 4 + 2]);
              predPathMsg.poses[k] = pose;
            }
            pubPred.publish(predPathMsg);
          }
          else
          {
            ROS_WARN_THROTTLE(1.0, "mpcc solve failed");
          }
        }
      }

  if (goalReached)
  {
    vxCmd = 0.0;
    vyCmd = 0.0;
    wCmd = 0.0;
  }

  if (safetyStop >= 1 && !alignActive)
  {
    vxCmd = 0.0;
        vyCmd = 0.0;
      }
      if (safetyStop >= 2)
      {
        wCmd = 0.0;
      }

      targetCmd.linear.x = vxCmd;
      targetCmd.linear.y = vyCmd;
      targetCmd.angular.z = wCmd;

    }
    // Publish smoothed command at loop rate
    {
      geometry_msgs::Twist cmd_out = targetCmd;
      if (enableCmdSmoothing)
      {
        const double now = ros::Time::now().toSec();
        double dt = now - lastCmdTime;
        if (dt <= 0.0 || dt > 1.0)
        {
          dt = std::max(1e-3, 1.0 / 100.0);
          lastCmdTime = now;
          lastCmd = cmd_out;
        }
        else
        {
          const double alpha = clampVal(dt / (cmdSmoothTau + dt), 0.0, 1.0);
          geometry_msgs::Twist filt = cmd_out;
          filt.linear.x = lastCmd.linear.x + alpha * (cmd_out.linear.x - lastCmd.linear.x);
          filt.linear.y = lastCmd.linear.y + alpha * (cmd_out.linear.y - lastCmd.linear.y);
          filt.angular.z = lastCmd.angular.z + alpha * (cmd_out.angular.z - lastCmd.angular.z);

          const double maxDv = cmdMaxAccelLin * dt;
          const double maxDw = cmdMaxAccelAng * dt;
          filt.linear.x = lastCmd.linear.x + clampVal(filt.linear.x - lastCmd.linear.x, -maxDv, maxDv);
          filt.linear.y = lastCmd.linear.y + clampVal(filt.linear.y - lastCmd.linear.y, -maxDv, maxDv);
          filt.angular.z = lastCmd.angular.z + clampVal(filt.angular.z - lastCmd.angular.z, -maxDw, maxDw);

          cmd_out = filt;
          lastCmd = cmd_out;
          lastCmdTime = now;
        }
      }

      pubSpeed.publish(cmd_out);
      yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
      yhs_cmd_vel.ctrl_cmd_gear = 6;
      yhs_cmd_vel.ctrl_cmd_x_linear = cmd_out.linear.x;
      yhs_cmd_vel.ctrl_cmd_z_angular = cmd_out.angular.z * 57.29d;
      yhs_ctrl_pub.publish(yhs_cmd_vel);
    }

    rate.sleep();
  }
  return 0;
}
