#ifndef NASH_H
#define NASH_H
#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>
#include "helper.h"
#include <numeric>
#include <atomic>
#include <thread>

// set of functions that compute nash equilibria of a given normal form (A,B)
// representation of a game
// might want to investigate value matrix pruning: 
//  removing completely dominated actions
namespace nash {

  // returns true if at least one pure nash equilibrium was found
  // returns by reference a vector of 2 vectors that represent 
  //  indices (row,col) of locations of pure nash equilibria
  // operates on rows: for each row, find the column of highest payoff for P2 
  //    for each of the column of highest payoff, check the row of highest payoff for P1
  //    if these match, the (row,col) is a pure nash equilibrium
  // This arises simply from the definition of pure nash equilibria
  bool pureNashEquilibrium(std::vector<std::vector<int>>& idx, 
                            const Eigen::MatrixXd& A, 
                            const Eigen::MatrixXd& B);

  // returns true if a mixed nash equilibrium was found
  // returns by reference a vector of Eigen::Vector2d, representing 
  //  the mixed strategies of player 1 and 2
  //  seems like with these implementations LH is faster for finding 
  //  NE in very large (>500x500) matrices
  //
  // performs a brute-force multi-threaded implementation of lemkeHowson
  //  that starts each thread with a different pivot variable
  //  thread synchronization is via an atomic<bool> flag
  bool mixedNashEquilibrium(std::vector<Eigen::VectorXd>& mixed_strategy,
                            const Eigen::MatrixXd& A, 
                            const Eigen::MatrixXd& B);

  //  Lemke-Howson pivot algorithm, ~~but looking into the more modern 
  //  and potentially faster Porter, Nudelman, Shoham algorithm~~
  //  default is a standard, single threaded LH alg,
  //  optional pivot var start 
  //  optional pointer to atomic<bool> that facilitates 
  //      the multithreaded implementation 
  bool lemkeHowson(std::vector<Eigen::VectorXd>& mixed_strategy,
                            const Eigen::MatrixXd& A_src, 
                            const Eigen::MatrixXd& B_src,
                            int pivot_var=0,
                            std::atomic<bool>* has_been_written= nullptr
                            );

  // implementation of sorted support enumeration algorithm described by
  //  Porter, Nudelman, Shoham
  bool pns(std::vector<Eigen::VectorXd>& mixed_strategy,
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B );

  // given a vector of vectors of indices that represent nash equilibria 
  //  of the game G(A,B), return the index of the risk dominant equilibrium
  int riskDominantEquilibrium(const std::vector<std::vector<int>>& equilibria,
                              const Eigen::MatrixXd& A, 
                              const Eigen::MatrixXd& B);
                        
}

#endif
