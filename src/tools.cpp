#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse = VectorXd(4);
  rmse.setZero();

  if (estimations.empty()) {
    cout << "ERROR CalculateRMSE: estimations vector is empty" << endl;
    return rmse;
  }

  if (ground_truth.empty()) {
    cout << "ERROR CalculateRMSE: ground-truth vector is empty" << endl;
    return rmse;
  }

  unsigned long estimation_size = estimations.size();
  if (estimation_size != ground_truth.size()) {
    cout << "ERROR CalculateRMSE: ground-truth vector size must be same as estimations vectors size" << endl;
    return rmse;
  }

  for (unsigned int i = 0; i < estimation_size; ++i) {
    VectorXd diff = estimations[i] - ground_truth[i];
    diff = diff.array() * diff.array();
    rmse += diff;
  }

  rmse = rmse / estimation_size;
  rmse = rmse.array().sqrt();
  return rmse;
}