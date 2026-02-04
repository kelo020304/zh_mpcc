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
double vsMin = 0.0;
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

bool gearAutoEnable = true;
double rotatePathLenThre = 0.05;
double rotateDsThre = 0.02;
double rotateYawThreDeg = 20.0;
double rotateYawKp = 2.0;
double rotateWMax = 1.0;

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

static geometry_msgs::Twist targetCmd;
static geometry_msgs::Twist lastCmd;
static double lastCmdTime = 0.0;

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

static void computePathYawAndS()
{
  const int n = static_cast<int>(pathX.size());
  pathYaw.assign(n, 0.0);
  pathS.assign(n, 0.0);
  if (n <= 1) return;

  for (int i = 0; i < n - 1; i++)
  {
    double dx = pathX[i + 1] - pathX[i];
    double dy = pathY[i + 1] - pathY[i];
    pathYaw[i] = atan2(dy, dx);
    pathS[i + 1] = pathS[i] + hypot(dx, dy);
  }
  pathYaw[n - 1] = pathYaw[n - 2];

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
    pathS[0] = 0.0;
    for (int i = 1; i < pathSize; i++)
    {
      double dx = pathX[i] - pathX[i - 1];
      double dy = pathY[i] - pathY[i - 1];
      double ds = hypot(dx, dy);
      pathS[i] = pathS[i - 1] + ds;
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
    computePathYawAndS();
  }
  pathInit = true;
  pathJustUpdated = true;
}

static void samplePathByS(double s, double &x, double &y, double &yaw)
{
  const int n = static_cast<int>(pathS.size());
  if (n <= 1)
  {
    x = pathX.empty() ? 0.0 : pathX[0];
    y = pathY.empty() ? 0.0 : pathY[0];
    yaw = pathYaw.empty() ? 0.0 : pathYaw[0];
    return;
  }
  if (s <= pathS.front())
  {
    x = pathX.front();
    y = pathY.front();
    yaw = pathYaw.front();
    return;
  }
  if (s >= pathS.back())
  {
    x = pathX.back();
    y = pathY.back();
    yaw = pathYaw.back();
    return;
  }
  int idx = 0;
  while (idx + 1 < n && pathS[idx + 1] < s) idx++;
  const double s0 = pathS[idx];
  const double s1 = pathS[idx + 1];
  const double t = (s - s0) / std::max(1e-6, s1 - s0);
  x = pathX[idx] * (1.0 - t) + pathX[idx + 1] * t;
  y = pathY[idx] * (1.0 - t) + pathY[idx + 1] * t;
  yaw = pathYaw[idx] * (1.0 - t) + pathYaw[idx + 1] * t;
}

struct QPSolution
{
  std::vector<double> xk; // size NX*(N+1)
  std::vector<double> uk; // size NU*N
};

static bool solveMpccQp(
    const std::vector<double> &x0,
    const std::vector<double> &s_guess,
    int N,
    double dt,
    double qVsEff,
    double vsMinEff,
    double vsMaxEff,
    int gearSign,
    QPSolution &sol)
{
  const int NX = 4;
  const int NU = 4;

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

  for (int k = 0; k <= N; k++)
  {
    double x_ref = 0.0, y_ref = 0.0, yaw_ref = 0.0;
    samplePathByS(s_guess[k], x_ref, y_ref, yaw_ref);

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

    // Q for x,y from contour/lag errors
    const double qxx = 2.0 * (wC * a_cx * a_cx + wL * a_lx * a_lx);
    const double qxy = 2.0 * (wC * a_cx * a_cy + wL * a_lx * a_ly);
    const double qyy = 2.0 * (wC * a_cy * a_cy + wL * a_ly * a_ly);

    Q[k][0 + 0 * NX] += qxx;
    Q[k][0 + 1 * NX] += qxy;
    Q[k][1 + 0 * NX] += qxy;
    Q[k][1 + 1 * NX] += qyy;

    q[k][0] += 2.0 * (wC * b_c * a_cx + wL * b_l * a_lx);
    q[k][1] += 2.0 * (wC * b_c * a_cy + wL * b_l * a_ly);

    // yaw tracking
    Q[k][2 + 2 * NX] += 2.0 * wYaw;
    q[k][2] += 2.0 * wYaw * (-yaw_ref);

    if (k < N)
    {
      R[k][0 + 0 * NU] += 2.0 * rVx;
      R[k][1 + 1 * NU] += 2.0 * rVy;
      R[k][2 + 2 * NU] += 2.0 * rW;
      R[k][3 + 3 * NU] += 2.0 * rVs;
      r[k][3] += -qVsEff;
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

  for (int k = 0; k < N; k++)
  {
    idxbu[k] = {0, 1, 2, 3};
    lbu[k] = {-vxMax, -vyMax, -wMax, vsMinEff};
    ubu[k] = {vxMax, vyMax, wMax, vsMaxEff};
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
  // Use NULL for unused constraints instead of arrays of nullptr
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
  nhPrivate.param("vs_min", vsMin, 0.0);
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
  nhPrivate.param("stop_dis_thre", stopDisThre, 0.2);
  nhPrivate.param("align_dist_thre", alignDistThre, alignDistThre);
  nhPrivate.param("align_pos_thre", alignPosThre, alignPosThre);
  nhPrivate.param("align_pos_speed", alignPosSpeed, alignPosSpeed);
  nhPrivate.param("align_yaw_thre_deg", alignYawThreDeg, alignYawThreDeg);
  nhPrivate.param("align_yaw_speed", alignYawSpeed, alignYawSpeed);
  nhPrivate.param("align_timeout_s", alignTimeoutS, alignTimeoutS);
  nhPrivate.param("gear_auto_enable", gearAutoEnable, gearAutoEnable);
  nhPrivate.param("rotate_path_len_thre", rotatePathLenThre, rotatePathLenThre);
  nhPrivate.param("rotate_ds_thre", rotateDsThre, rotateDsThre);
  nhPrivate.param("rotate_yaw_thre_deg", rotateYawThreDeg, rotateYawThreDeg);
  nhPrivate.param("rotate_yaw_kp", rotateYawKp, rotateYawKp);
  nhPrivate.param("rotate_w_max", rotateWMax, rotateWMax);

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
            double goalHeading = atan2(dyg, dxg);
            double headingBody = wrapAngle(goalHeading - vehicleYaw);
            vxCmd = alignPosSpeed * cos(headingBody);
            vyCmd = alignPosSpeed * sin(headingBody);
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
          // find nearest s
          int bestIdx = 0;
          double bestDis = 1e9;
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
          double s0 = pathS.empty() ? 0.0 : pathS[bestIdx];

          int gearSign = 1;
          bool rotateGear = false;
          double vsMinEff = vsMin;
          double vsMaxEff = vsMax;
          double qVsEff = qVs;
          if (gearAutoEnable && !pathYaw.empty())
          {
            const double pathYawRef = pathYaw[bestIdx];
            const double headingErr = wrapAngle(pathYawRef - vehicleYawRel);
            gearSign = (cos(headingErr) >= 0.0) ? 1 : -1;

            if (!pathS.empty() && pathS.back() < rotatePathLenThre)
            {
              rotateGear = true;
            }
            else if (bestIdx + 1 < pathSize)
            {
              const double ds = hypot(pathX[bestIdx + 1] - pathX[bestIdx],
                                      pathY[bestIdx + 1] - pathY[bestIdx]);
              if (ds < rotateDsThre && fabs(headingErr) * 180.0 / PI > rotateYawThreDeg)
              {
                rotateGear = true;
              }
            }
          }

          if (rotateGear && !pathYaw.empty())
          {
            const double yawErr = wrapAngle(pathYaw[bestIdx] - vehicleYawRel);
            vxCmd = 0.0;
            vyCmd = 0.0;
            wCmd = clampVal(rotateYawKp * yawErr, -rotateWMax, rotateWMax);
            ROS_INFO_THROTTLE(1.0, "\033[36mgear=rotate\033[0m");
          }
          else
          {
            if (gearSign > 0)
            {
              vsMinEff = std::max(0.0, vsMin);
              vsMaxEff = vsMax;
            }
            else
            {
              vsMinEff = vsMin;
              vsMaxEff = std::min(0.0, vsMax);
            }
            qVsEff = qVs * static_cast<double>(gearSign);
            ROS_INFO_THROTTLE(1.0, "\033[36mgear=%s\033[0m", (gearSign > 0) ? "forward" : "reverse");

          int N = std::max(1, mpcHorizon);
          vector<double> s_guess(N + 1, s0);
          if (gearSign > 0)
          {
            double step_s = std::max(0.0, vsMinEff) * mpcDt;
            for (int k = 1; k <= N; k++)
              s_guess[k] = s_guess[k - 1] + std::max(step_s, 0.05);
          }
          else
          {
            double step_s = std::min(0.0, vsMinEff) * mpcDt;
            for (int k = 1; k <= N; k++)
              s_guess[k] = s_guess[k - 1] + std::min(step_s, -0.05);
          }
          if (!last_s_guess.empty() && static_cast<int>(last_s_guess.size()) == N + 1 && !pathJustUpdated)
          {
            s_guess = last_s_guess;
            s_guess[0] = s0;
            if (gearSign > 0)
            {
              for (int k = 1; k <= N; k++)
                s_guess[k] = std::max(s_guess[k], s_guess[k - 1]);
            }
            else
            {
              for (int k = 1; k <= N; k++)
                s_guess[k] = std::min(s_guess[k], s_guess[k - 1]);
            }
          }
          pathJustUpdated = false;

          vector<double> x0 = {vehicleXRel, vehicleYRel, vehicleYawRel, s0};
          QPSolution sol;
          if (solveMpccQp(x0, s_guess, N, mpcDt, qVsEff, vsMinEff, vsMaxEff, gearSign, sol))
          {
            double speedScale = std::max(0.0f, std::min(1.0f, joySpeed));
            if (autonomyMode && joySpeedRaw == 0)
            {
              speedScale = std::max(0.0, std::min(1.0, autonomySpeed / std::max(1.0, maxSpeed)));
            }
            vxCmd = sol.uk[0] * speedScale;
            vyCmd = sol.uk[1] * speedScale;
            wCmd = sol.uk[2] * speedScale;
            ROS_INFO_THROTTLE(1.0, "\033[36mmpcc cmd: vx=%.3f vy=%.3f w=%.3f\033[0m",
                              vxCmd, vyCmd, wCmd);
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
