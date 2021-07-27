#ifndef COST_CALCULATOR_H
#define COST_CALCULATOR_H
#include "sync_future.h"
#include <vector>
#include <thread>
#include <functional>
#include "helper.h"

// class that calculates resulting future costs in a parallel manner
// to prevent the number of threads exploding, threads operate on 
// individual other futures, and iterate through possible self futures
// so that there are only x_other_futures_.size() number of threads, 
// rather than squared many
// Chose to operate per other_future, since Eigen::Matrix defaults to 
// column-major order for memory allocation, so accessing large columns
// at a time _should_ be faster than for rows.

// futures are contained in the syncFuture class to safely provide synchronous access
// form of cost function to be passed
using costFunction = std::function<void(double&,
                                        const Eigen::VectorXd&, // x_self
                                        const Eigen::VectorXd&, // u_self
                                        const Eigen::VectorXd&, // x_other
                                        const Eigen::VectorXd&  // u_other
                                        )>;
class costCalculator
{ 
  // preallocated matrix of costs to be popluated
  // matrix must be of size (x_self_futures_.size(),x_other_futures_.size())
  // cost function is specifically for self cost. To calculate the other player's
  //  cost matrix requires another futureCombinator instance, with switched definitions
  //  of self and other
  Eigen::MatrixXd& self_cost_matrix_;

  // remember u futures are of length K, 
  //          x futures are of length K+1 and start with x0
  // these are in the process of being populated by futureCombinator self player
  std::vector<syncFuture>& x_self_futures_;
  std::vector<syncFuture>& u_self_futures_;

  // these are in the process of being populated by futureCombinator other player
  std::vector<syncFuture>& x_other_futures_;
  std::vector<syncFuture>& u_other_futures_;

  int& K_i_; 
  int& K_steps_; 
  const double& dt_;
  
  std::vector<std::thread> vector_threads_;
  costFunction& jCostPtrSelf_;

  public:

  public:
  costCalculator( 
      Eigen::MatrixXd& cost_matrix,
      std::vector<syncFuture>& X_s,
      std::vector<syncFuture>& U_s,
      std::vector<syncFuture>& X_o,
      std::vector<syncFuture>& U_o,
      int& K_start, 
      int& K_steps, 
      const double& t_step, 
      costFunction& jCostPtr
      ):
    self_cost_matrix_(cost_matrix),
    x_self_futures_(X_s), u_self_futures_(U_s),
    x_other_futures_(X_o), u_other_futures_(U_o),
    K_i_(K_start), K_steps_(K_steps),
    dt_(t_step), jCostPtrSelf_(jCostPtr)
  {
    reserve_thread_vector();
  };

  void reserve_thread_vector();

  void start_calc_costs();

  void join_threads();


  private:

  // operates on all x/u_self_futures_, 
  // and on x/u_other_futures_[col_idx]
  // writing to matrix entries (1:end,col_idx)
  // 
  //    this is sketchy, since the matrix values are written by many threads at the same time,
  //    but they _should_ not result in concurrency errors
  //      https://en.cppreference.com/w/cpp/container#Thread_safety
  void thread_runner(const int col_idx);

  // assumes cumulative cost, with jCostPtrSelf_ calculating cost at each timestep
  void calcFutureCost(double& cost_total,
                      const std::vector<Eigen::VectorXd>& x_future_self,
                      const std::vector<Eigen::VectorXd>& u_future_self,
                      const std::vector<Eigen::VectorXd>& x_future_other,
                      const std::vector<Eigen::VectorXd>& u_future_other );

};
#endif
