#include "nash_controller.h"

void nashController::start_calculations()
{
  other_futures_.start_calc_futures();
  self_futures_.start_calc_futures();

  other_cost_calc_.start_calc_costs();
  self_cost_calc_.start_calc_costs();
}

void nashController::join_calculations()
{
  pool_futures_.process_jobs();
  self_cost_calc_.join_threads();
  other_cost_calc_.join_threads();
}

void nashController::reserve_memory()
{
  if (  !x_futures_self_.size()  ||
      !u_futures_self_.size()  ||
      !x_futures_other_.size() ||
      !u_futures_other_.size()    )
  {
    std::cerr << "Warn: attempt to allocate syncFuture memory without preallocating vectors\n";
    x_futures_self_ = 
      std::vector<syncFuture>(std::pow(n_self_,K_steps_));
    u_futures_self_ = 
      std::vector<syncFuture>(std::pow(n_self_,K_steps_));
    x_futures_other_ = 
      std::vector<syncFuture>(std::pow(n_other_,K_steps_));
    u_futures_other_ = 
      std::vector<syncFuture>(std::pow(n_other_,K_steps_));

    cost_matrix_self_ = Eigen::MatrixXd(
        (int)std::pow(n_self_,K_steps_),
        (int)std::pow(n_other_,K_steps_));
    cost_matrix_other_ = Eigen::MatrixXd(
        (int)std::pow(n_other_,K_steps_),
        (int)std::pow(n_self_,K_steps_));

    self_cost_calc_.reserve_thread_vector(); 
    other_cost_calc_.reserve_thread_vector(); 
  }

  for(auto& x_future : x_futures_self_)
  {
    x_future.reserve(K_steps_+1,x_dim_);
  }
  for(auto& u_future : u_futures_self_)
  {
    u_future.reserve(K_steps_,u_dim_);
  }
  for(auto& x_future : x_futures_other_)
  {
    x_future.reserve(K_steps_+1,x_dim_);
  }
  for(auto& u_future : u_futures_other_)
  {
    u_future.reserve(K_steps_,u_dim_);
  }

}

Eigen::VectorXd nashController::calcControlInput() 
{
  Eigen::VectorXd u_opt(u_dim_);

  start_calculations();
  join_calculations();

  std::vector<std::vector<int>> pure_nash_eq;
  bool pure_nash = nash::pureNashEquilibrium( 
      pure_nash_eq, 
      value_matrix_self(), 
      value_matrix_other());
  if (pure_nash)
  {
    if (pure_nash_eq.size()==1)
    { u_opt = u_futures_self_[pure_nash_eq[0][0]].data_[0]; }
    else
    { 
      std::vector<int> eq = pure_nash_eq[nash::riskDominantEquilibrium(
                                              pure_nash_eq, 
                                              value_matrix_self(), 
                                              value_matrix_other())];
      u_opt = u_futures_self_[eq[0]].data_[0];
    }
  }
  else
  {
    std::vector<Eigen::VectorXd> mix_nash_eq;
    bool mix_nash = nash::mixedNashEquilibrium( mix_nash_eq,
        value_matrix_self(),
        value_matrix_other() );
    if (mix_nash)
    {
      helper::print(mix_nash_eq[0].transpose());
      helper::print(mix_nash_eq[1].transpose());
      int idx = draw_from_distr(mix_nash_eq[0]);
      u_opt = u_futures_self_[idx].data_[0];
    }
  }
  return u_opt;
}

void nashController::resetNextTimeStep() 
{ // assumes K_i and x_0 are updated outside via references
  // reset flags on futures, (clear futures?)
  for (syncFuture& x:x_futures_self_)
  { x.reset_failsafe(); }
  for (syncFuture& u:u_futures_self_)
  { u.reset_failsafe(); }
  for (syncFuture& x:x_futures_other_)
  { x.reset_failsafe(); }
  for (syncFuture& u:u_futures_other_)
  { u.reset_failsafe(); }
  // set cost values to zero again (remember, these are cumulatively calculated)
  cost_matrix_self_.setZero();
  cost_matrix_other_.setZero();
}

int nashController::draw_from_distr(Eigen::VectorXd pdf)
{
  Eigen::VectorXd cdf(pdf.size());
  cdf[0]=pdf[0];
  for (int i=1; i<cdf.size(); i++)
  { cdf[i] = cdf[i-1]+pdf[i]; }
  // get random number
  std::random_device rand_dev;
  std::mt19937 generator(rand_dev());
  std::uniform_real_distribution<double> distr(0,1);
  double draw = distr(generator);
  // find location in cdf
  int idx;
  for(idx=0; idx<cdf.size(); idx++)
  {
    if(draw<cdf[idx])
    {break;}
  }
  return idx;
}

Eigen::MatrixXd nashController::value_matrix_self()
{
  return -cost_matrix_self_;
}
Eigen::MatrixXd nashController::value_matrix_other()
{
  return -cost_matrix_other_.transpose();
}

