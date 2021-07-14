#ifndef HELPER_H
#define HELPER_H

#include <Eigen/Core>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

// class that protects the data member with a shared mutex and condition variable flagging
namespace helper
{
  void print(const Eigen::MatrixXd& M);
  void printToMatlab(const Eigen::MatrixXd& M);
  void printToMatlab(const Eigen::MatrixXd& M,std::string name);
}
#endif
