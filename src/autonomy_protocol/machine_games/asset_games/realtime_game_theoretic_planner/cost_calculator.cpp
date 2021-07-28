#include "cost_calculator.h"

void costCalculator::reserve_thread_vector()
{
  if(!self_cost_matrix_.cols())
    //std::cerr << "Warning: cost matrix not initialized yet";

  vector_threads_ = std::vector<std::thread>(self_cost_matrix_.cols());
}

void costCalculator::start_calc_costs()
{
  for (int col=0; col<self_cost_matrix_.cols(); col++)
  {
    vector_threads_[col] = std::thread(
        &costCalculator::thread_runner, this,
        col);
  }
}

void costCalculator::join_threads()
{
  for (std::thread& th : this->vector_threads_)
  {
    if(th.joinable())
      th.join();
  }
}


void costCalculator::thread_runner(const int col_idx)
{
  // get read lock on column
  // this is necessary for all following calcs, so do it first
  auto lk_x_other = this->x_other_futures_[col_idx].wait_read();
  auto lk_u_other = this->u_other_futures_[col_idx].wait_read();
  // wait sequentially on read locks for self futures
  for (int r=0; r<x_self_futures_.size(); r++)
  {
    // get read lock on row at a time
    auto lk_x_self = this->x_self_futures_[r].wait_read();
    auto lk_u_self = this->u_self_futures_[r].wait_read();

    // calculate cost and place in matrix
    calcFutureCost(this->self_cost_matrix_(r,col_idx),
        this->x_self_futures_[r].data_,
        this->u_self_futures_[r].data_,
        this->x_other_futures_[col_idx].data_,
        this->u_other_futures_[col_idx].data_);
    lk_x_self.unlock();
    lk_u_self.unlock();
  }
  lk_x_other.unlock();
  lk_u_other.unlock();
}

void costCalculator::calcFutureCost(double& cost_total,
    const std::vector<Eigen::VectorXd>& x_future_self,
    const std::vector<Eigen::VectorXd>& u_future_self,
    const std::vector<Eigen::VectorXd>& x_future_other,
    const std::vector<Eigen::VectorXd>& u_future_other )
{
  int K = this->K_steps_;
  for(int i=0; i < K; i++)
  {
    (this->jCostPtrSelf_)(cost_total,
        x_future_self[i+1],
        u_future_self[i],
        x_future_other[i+1],
        u_future_other[i]
        );
  }
}
