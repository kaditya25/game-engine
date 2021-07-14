#include "future_combinator.h"
#include "helper.h"

void futureCombinator::start_calc_futures()
{ int n = u_enumerated_.size();
  int K = K_steps_;

  // index vector for counting permutations
  std::vector<int> idx_vector(K);

  // keep track of thread task grouping
  int idx = 0;

  //std::cout << "calculating combinations of n=" << n << ", K= " << K<<  "\n";

  for (int cnt=0; cnt < std::pow(n,K); cnt++)
  {
    {
      std::lock_guard<std::shared_timed_mutex> lk_u(u_futures_[cnt].m_);
      // get control future from permutation 
      for (int i=0; i < K; i++)
      { u_futures_[cnt].data_[i] = u_enumerated_[idx_vector[i]]; } 
      u_futures_[cnt].write_complete();
      u_futures_[cnt].failsafe_ = true;
    }
    u_futures_[cnt].notify_all();

    // assign initial condition
    {
      auto lk_x = std::unique_lock<std::shared_timed_mutex>(x_futures_[cnt].m_);
      x_futures_[cnt].data_[0] = x_0_;
      lk_x.unlock();
    }

    // group in game_state_future struct
    vector_game_state_futures_[cnt] = 
      gameStateFuture(&fDynPtr_,
          &x_futures_[cnt],
          &u_futures_[cnt],
          &dt_, &K_i_, &K_steps_);
    // thread grouping
    if (cnt % number_of_tasks_per_thread_ ==
        number_of_tasks_per_thread_-1)
    {
      pool_.add_job(std::bind(
            &futureCombinator::thread_runner, this, 
            std::begin(vector_game_state_futures_)+idx,
            std::begin(vector_game_state_futures_)+cnt+1   ));
      idx = cnt+1;
    }

    // calculate next permutation
    for (int pos = K-1; pos>=0; pos--)
    {
      idx_vector[pos]++;
      if(idx_vector[pos]==n)
        idx_vector[pos] = 0;
      else break;
    }
  }
  // handle remainder cases
  pool_.add_job(std::bind(
        &futureCombinator::thread_runner, this, 
        std::begin(vector_game_state_futures_)+idx,
        std::end(vector_game_state_futures_)         ));

}

void futureCombinator::thread_runner(
    std::vector<gameStateFuture>::iterator begin,
    std::vector<gameStateFuture>::iterator end)
{
  int calc_no = 1;
  while (begin!=end)
  {
    futureCombinator::simFuture(*begin);
    begin++;
  }
}


void futureCombinator::simFuture( gameStateFuture& gsf)
{
  auto lk_x = gsf.x_future->get_write_lock();
  for(int i=1; i <= *gsf.K_steps; i++)
  { 
    (*gsf.fDyn)((gsf.x_future)->data_[i], (gsf.x_future)->data_[i-1], (gsf.u_future)->data_[i-1], *gsf.dt); 
  }
  (*gsf.x_future).release_write_lock(lk_x);
  // notify again
  (*gsf.x_future).write_complete();
  (*gsf.x_future).notify_all();
}
