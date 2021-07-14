#ifndef FUTURE_COMBINATOR_H
#define FUTURE_COMBINATOR_H
#include "sync_future.h"
#include <vector>
#include <thread>
#include "thread_pool.h"

// class that calculates possible control futures and state futures in a parallel manner
// futures are contained in the syncFuture class to safely provide synchronous access
// form of dynamics function to be passed
using dynFunction = std::function<void(Eigen::VectorXd&, // x_k+1
    const Eigen::VectorXd&, // x_k
    const Eigen::VectorXd&, // u
    const double&           // dt
    )>;
class futureCombinator
{ 
  struct gameStateFuture {
    public:
      gameStateFuture(){};
      gameStateFuture( dynFunction* f, 
          syncFuture* x,
          syncFuture* u,
          const double* dt_input,
          const int* K_start_input,
          const int* K_steps_input )
        : fDyn(f), x_future(x), u_future(u), dt(dt_input), K_start(K_start_input), K_steps(K_steps_input){}
      dynFunction* fDyn;
      syncFuture* x_future;
      syncFuture* u_future;
      const double* dt;
      const int* K_start;
      const int* K_steps;
  };
  //  notation shortcuts:
  //  x_future_[0] is the initial condition, but u_future_[0] is the first control input
  //  time is K=[K_i,K_f], so n_k = K_f-K_i+1 total timesteps
  //  
  //  timeline:
  //  (K_i-1) | K_i  | K_i+1 | ... |  K_f-1  |   K_f
  //   x[0]   | x[1] | x[2]  | ... | x[n_k]  | x[n_k+1]
  //   u[0]   | u[1] | u[2]  | ... | u[n_k]  | 
  //
  //   where x[k+1] = f(x[k],u[k])
  // 
  // these references are populated by start_calc_futures()
  std::vector<syncFuture>& x_futures_;
  std::vector<syncFuture>& u_futures_;

  std::vector<Eigen::VectorXd>& u_enumerated_;
  int& K_i_; 
  int& K_steps_; 
  const double& dt_;
  Eigen::VectorXd& x_0_;
  const int number_of_tasks_per_thread_;
  //std::vector<std::thread> vector_threads_;
  dynFunction& fDynPtr_;
  std::vector<gameStateFuture> vector_game_state_futures_;

  threadPool& pool_;

  public:
  futureCombinator( 
      std::vector<syncFuture>& X,
      std::vector<syncFuture>& U,
      std::vector<Eigen::VectorXd>& u_enum, 
      int& K_start, 
      int& K_steps, 
      const double& t_step, 
      Eigen::VectorXd& x_init,
      dynFunction& fDynPtr ,
      threadPool& pool,
      const int number_tasks = 20  ):
    x_futures_(X), u_futures_(U),
    u_enumerated_(u_enum), K_i_(K_start), K_steps_(K_steps),
    dt_(t_step), x_0_(x_init), fDynPtr_(fDynPtr), number_of_tasks_per_thread_(number_tasks),
    pool_(pool)
  {
    // keep track of all game state futures
    vector_game_state_futures_ = std::vector<gameStateFuture>(std::pow( u_enumerated_.size(), K_steps_));
  };

  void start_calc_futures();

  private:

  void thread_runner(
      std::vector<gameStateFuture>::iterator begin,
      std::vector<gameStateFuture>::iterator end);

  void simFuture( gameStateFuture& gsf);

};
#endif
