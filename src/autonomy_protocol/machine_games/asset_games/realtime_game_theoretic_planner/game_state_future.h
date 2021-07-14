#ifndef GAME_STATE_FUTURE_H
#define GAME_STATE_FUTURE_H
#include "sync_future.h"
#include <vector>

using dynFunction = std::function<void(Eigen::VectorXd&, // x_k+1
    const Eigen::VectorXd&, // x_k
    const Eigen::VectorXd&, // u
    const double&           // dt
    )>;

struct gameStateFuture {
  public:
    gameStateFuture(){};
    gameStateFuture( dynFunction* f, 
        syncFuture* x,
        syncFuture* u,
        const double* dt_input,
        const int* K_start_input,
        const int* K_end_input )
      : fDyn(f), x_future(x), u_future(u), dt(dt_input), K_start(K_start_input), K_end(K_end_input){}
    dynFunction* fDyn;
    syncFuture* x_future;
    syncFuture* u_future;
    const double* dt;
    const int* K_start;
    const int* K_end;
};

#endif

