#include "emplanner/reference_line/reference_line_smoother.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include "eigen3/Eigen/Eigen"

//使用：将类的初始化参数 传递给 成员变量
ReferenceLineSmoother::ReferenceLineSmoother(
    const ReferenceLineSmootherConfig &config)
    : config_(config) {}

// 默认构造函数
ReferenceLineSmoother::ReferenceLineSmoother() {}

void ReferenceLineSmoother::Smooth(const std::vector<MapPoint> &raw_reference_points,
                                   std::vector<MapPoint> &smoothed_reference_points) {

  // 初始化平滑后的参考线
  smoothed_reference_points = raw_reference_points;
  
  // 提取原始参考点的x,y坐标
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<std::pair<double, double>> smoothed_point2d;
  for (const auto &point : raw_reference_points) {
    raw_point2d.emplace_back(point.x, point.y);
  }
  
  // 调用路径平滑算法
  DiscretePointsSmooth(raw_point2d, &smoothed_point2d);

  // 构建平滑后的参考点
  for (size_t i = 0; i < smoothed_point2d.size(); ++i) {
    smoothed_reference_points[i].x = smoothed_point2d[i].first;
    smoothed_reference_points[i].y = smoothed_point2d[i].second;
  }

  
}

// 离散点平滑
bool ReferenceLineSmoother::DiscretePointsSmooth(
    const std::vector<std::pair<double, double>> &raw_point2d,
    std::vector<std::pair<double, double>> *smoothed_point2d) {
  int n = raw_point2d.size();

  //初始化A1,A2,A3，f,lb,ub矩阵
  //平滑代价系数矩阵，x'A1'A1x, (n-2)
  Eigen::SparseMatrix<double> A1(2 * n, 2 * n);
  //路径长度代价矩阵 x'A2'A2x,(n-1)
  Eigen::SparseMatrix<double> A2(2 * n, 2 * n);
  //参考线偏离代价矩阵 x'A3'A3x,单位阵
  Eigen::SparseMatrix<double> A3(2 * n, 2 * n);

  Eigen::SparseMatrix<double> H(2 * n, 2 * n); //必须是稀疏矩阵
  Eigen::VectorXd f = Eigen::VectorXd::Zero(2 * n);
  Eigen::SparseMatrix<double> A(2 * n, 2 * n);
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(2 * n);
  Eigen::VectorXd ub = Eigen::VectorXd::Zero(2 * n);
  Eigen::VectorXd qp_solution = Eigen::VectorXd::Zero(2 * n);

  A.setIdentity();

  //赋值f,lb,ub;
  // MatrixXd下标从(0,0)开始,(1,2)表示第1行第2列
  for (int i = 0; i < n; i++) {
    f(2 * i) = raw_point2d[i].first;
    f(2 * i + 1) = raw_point2d[i].second;

    lb(2 * i) = f(2 * i) + config_.x_lower_bound;
    lb(2 * i + 1) = f(2 * i + 1) + config_.y_lower_bound;

    ub(2 * i) = f(2 * i) + config_.x_upper_bound;
    ub(2 * i + 1) = f(2 * i + 1) + config_.y_upper_bound;
  }

  //赋值A1
  for (int j = 0; j < n - 2; j++) {
    A1.insert(2 * j, 2 * j) = 1;
    A1.insert(2 * j, 2 * j + 2) = -2;
    A1.insert(2 * j, 2 * j + 4) = 1;
    A1.insert(2 * j + 1, 2 * j + 1) = 1;
    A1.insert(2 * j + 1, 2 * j + 3) = -2;
    A1.insert(2 * j + 1, 2 * j + 5) = 1;
  }
  //赋值A2
  for (int k = 0; k < n - 1; k++) {
    A2.insert(2 * k, 2 * k) = 1;
    A2.insert(2 * k, 2 * k + 2) = -1;
    A2.insert(2 * k + 1, 2 * k + 1) = 1;
    A2.insert(2 * k + 1, 2 * k + 3) = 1;
  }

  A3.setIdentity();

  H = 2 * (config_.weight_smooth * A1.transpose() * A1 +
           config_.weight_path_length * A2.transpose() * A2 +
           config_.weight_ref_deviation * A3);

  f = -2 * config_.weight_ref_deviation * f;

  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(2 * n);
  solver.data()->setNumberOfConstraints(2 * n);
  solver.data()->setHessianMatrix(H);
  solver.data()->setGradient(f);
  solver.data()->setLinearConstraintsMatrix(A);
  solver.data()->setLowerBound(lb);
  solver.data()->setUpperBound(ub);

  if (!solver.initSolver())
    return 0;
  if (!solver.solve())
    return 0;
  qp_solution = solver.getSolution();

  (*smoothed_point2d).resize(n);
  for (int i = 0; i < n; i++) {
    (*smoothed_point2d)[i].first = qp_solution(2 * i);
    (*smoothed_point2d)[i].second = qp_solution(2 * i + 1);
  }
}
