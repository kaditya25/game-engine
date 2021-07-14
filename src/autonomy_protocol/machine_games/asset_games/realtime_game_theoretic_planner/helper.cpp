#include "helper.h"

namespace helper
{

  void print(const Eigen::MatrixXd& M)
  { Eigen::IOFormat customFmt(3, 0, ", ", "\n", "[", "]");

    std::cout << M.format(customFmt) << "\n";
    return ;
  }

  void printToMatlab(const Eigen::MatrixXd& M)
  {
    Eigen::IOFormat customFmt(5, 0, ", ", "\n", "", ";");

    std::cout <<"[ "<< M.format(customFmt) <<"]" <<"\n";
    return ;
  }

  void printToMatlab(const Eigen::MatrixXd& M, std::string name)
  {
    std::cout << name << "= ";
    printToMatlab(M);
    return;
  }

}
