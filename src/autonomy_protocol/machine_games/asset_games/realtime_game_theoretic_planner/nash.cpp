#include "nash.h"

namespace nash
{
  // private helper functions

  namespace {
    // ~~~~ Lemke Howson ~~~~
    // perform a LH pivot to a matrix m 
    void pivot(Eigen::MatrixXf& m, int r, int c)
    {
      // equivalent row operation written as matrix algebra
      Eigen::VectorXf col = m.col(c); col[r]=0;
      m+= -1/m(r,c) *
        col * m.row(r) ;
    }

    // ~~~~ pns combinatorics ~~~~
    // turns out factorials of big numbers are big, who thunk it
    // calculate factorial: n!
    unsigned long factorial(const int n)
    {
      long fact = 1;
      for (int i = n; i >1; i--)
        fact = fact*i;
      return fact;
    }
    // calculate truncated factorial: n_max!/n_min!
    unsigned long factorial(const int n_max, const int n_min)
    {
      long fact = 1;
      for (int i = n_max; i >n_min; i--)
        fact = fact*i;
      return fact;
    }
    // get vector of indices of n-choose-k combinations 
    std::vector<std::vector<int>> nChoosek(const int n, const int k)
    {
      std::vector<std::vector<int>> combs(factorial(n,n-k)/factorial(k));
      long n_comb = 0;
      std::string perm(k,1); // get k ones
      perm.resize(n,0); // fill the front with zeros up to n size
      do
      {
        for (int i=0; i < n; ++i)
          if (perm[i]) combs[n_comb].push_back(i);
        n_comb++;
      } while (std::prev_permutation(perm.begin(),perm.end()));

      return combs;
    }

    // get all permutations of choose k values out of the 
    // lexicographically ordered vector vals
    std::vector<std::vector<int>> nChoosek(std::vector<int>vals , const int k)
    {
      int n = vals.size();
      std::vector<std::vector<int>> combs(factorial(n,k)/factorial(n-k));
      long n_comb = 0;
      std::string perm(k,1); // get k ones
      perm.resize(n,0); // fill the front with zeros up to n size
      do
      {
        for (int i=0; i < n; ++i)
          if (perm[i]) combs[n_comb].push_back(vals[i]);
        n_comb++;
      } while (std::prev_permutation(perm.begin(),perm.end()));

      return combs;
    }

    // feasibility function that checks and solves for a NE over the given supports 
    bool feasibility_function(
        std::vector<Eigen::VectorXd>& mixed_strategy,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& B,
        const std::vector<int>& support_p1,
        const std::vector<int>& support_p2  )
    {
      if ( support_p1.size() != support_p2.size() )
      {
        std::cerr << "warning: supports of different size are considered not uniquely solvable!\n";
        return false;
      }

      int m = support_p1.size();

      // solve for player 1 strategy
      Eigen::MatrixXd matrix_p1(m+1,m+1);
      Eigen::MatrixXd matrix_p2(m+1,m+1);
      // populate coefs
      for (int col = 0; col < m; ++col)
      {
        for (int row = 0; row < m; ++row)
        {
          matrix_p1(row,col) = 
            B(support_p1[col],support_p2[row]);
          matrix_p2(row,col) = 
            A(support_p1[row],support_p2[col]);
        }
      }
      // all sum to expected value
      matrix_p1.rightCols<1>() = -Eigen::VectorXd::Ones(m+1);
      matrix_p2.rightCols<1>() = -Eigen::VectorXd::Ones(m+1);
      // sum of probabilities equals 1
      matrix_p1.bottomRows<1>() = Eigen::VectorXd::Ones(m+1);
      matrix_p2.bottomRows<1>() = Eigen::VectorXd::Ones(m+1);
      // except for expected value
      matrix_p1(m,m) = 0;
      matrix_p2(m,m) = 0;

      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(m+1);
      rhs[m] = 1;

      Eigen::VectorXd p1_sol = 
        matrix_p1.colPivHouseholderQr().solve(rhs);
      Eigen::VectorXd p2_sol = 
        matrix_p2.colPivHouseholderQr().solve(rhs);

      Eigen::VectorXd p1_strat = Eigen::VectorXd::Zero(A.rows());
      Eigen::VectorXd p2_strat = Eigen::VectorXd::Zero(A.cols());

      for (int i=0; i < m ; i++)
      { p1_strat[ support_p1[i] ] = p1_sol[i]; }
      for (int i=0; i < m ; i++)
      { p2_strat[ support_p2[i] ] = p2_sol[i]; }

      double p1_value = p2_sol[m];
      double p2_value = p1_sol[m];

      // check feasibility of solution:
      // all probabiliities greater than zero
      if ( (p1_strat.array()<0).any() || (p2_strat.array()<0).any() )
      {
        return false;
      }

      // no costs for other actions can be greater
      if ( ((A*p2_strat).array()  > p1_value+.001).any() ) 
      { 
        return false;
      }
      if ( (((B.transpose()*p1_strat).array()) > p2_value+.001).any()  )
      { 
        return false;
      }

      mixed_strategy.push_back(p1_strat);
      mixed_strategy.push_back(p2_strat);
      return true;
    }


  }


  bool pureNashEquilibrium(std::vector<std::vector<int>>& idx, 
      const Eigen::MatrixXd& A, 
      const Eigen::MatrixXd& B)
  {  
    if( idx.size()>0)
    {
      std::cerr << "nash: Error, idx not empty\n";
      return 0;
    }

    if( A.rows() != B.rows() || A.cols() != B.cols() )
    {
      std::cerr << "nash: Error, dimension mismatch\n";
      return 0;
    }

    Eigen::MatrixXd::Index max_col;
    double max_in_row, max_in_col;
    std::vector<int>v(2);
    for (int row=0; row<A.rows(); row++)
    {
      max_in_row = B.row(row).maxCoeff(&max_col);
      // maxCol returns the first instance of maximum value
      for (int col=max_col; col<A.cols(); col++)
      {
        if(B(row,col)==max_in_row) // this column is a maximum value for B
        {
          if(A(row,col)==A.col(col).maxCoeff()) // found nash equilibrium
          {v[0]=row; v[1]=col; idx.push_back(v);}
        }
      }
    }
    return idx.size();
  }

  bool mixedNashEquilibrium(std::vector<Eigen::VectorXd>& mixed_strategy,
                            const Eigen::MatrixXd& A, 
                            const Eigen::MatrixXd& B)
  { 
    // atomic bool that ensures only a single thread returns a soln
    std::atomic<bool> has_found_soln;
    has_found_soln = false;
    int num_threads = 20;
    int step = std::max((int)A.rows()/num_threads,1);

    // create a thread for each possible starting pivot var
    std::vector<std::thread> threads;
    for (int pivot = 0; pivot < A.rows(); pivot+=step)
    { // note that  mixed_strategy is passed via reference,
      // which would be rather thread unsafe without
      // careful synchronizing
      threads.emplace_back(  nash::lemkeHowson,
                              std::ref(mixed_strategy),
                              std::ref(A),std::ref(B),
                              pivot,
                              &has_found_soln );
    }
    // join threads
    for (auto& th : threads)
    {
      if (th.joinable())
        th.join();
    }
    return has_found_soln.load();
  }


  //  casts matrix to float, to account for matrices that are (very close to) degenerate, in order to get more resolution
  bool lemkeHowson(std::vector<Eigen::VectorXd>& mixed_strategy,
      const Eigen::MatrixXd& A_src,  
      const Eigen::MatrixXd& B_src, 
      int pivot_var,
      std::atomic<bool>* has_been_written
      )  
  {
    if( A_src.rows() != B_src.rows() || A_src.cols() != B_src.cols() )
    {
      std::cerr << "nash: Error, dimension mismatch\n";
      return 0;
    }
    Eigen::MatrixXf A,B;
    double min_val = std::min(A_src.minCoeff(),B_src.minCoeff());
    if (min_val < 0)
    {
      A = (A_src.array()-min_val).cast<float>();
      B = (B_src.array()-min_val).cast<float>();
    }
    else
    {
      A = (A_src).cast<float>();
      B = (B_src).cast<float>();
    }

    int m = A.rows(); int n = A.cols();
    // create tableaux, first corresponds to player 1 (B^Tx+s=1) constraint
    //    the second to player 2 (r+Ay=1) constraint
    std::vector<Eigen::MatrixXf> tableaux(2);
    tableaux[0].resize(n,m+n+1);
    tableaux[1].resize(m,m+n+1);

    tableaux[0] << B.transpose(), Eigen::MatrixXf::Identity(n,n), Eigen::VectorXf::Ones(n);
    tableaux[1] << Eigen::MatrixXf::Identity(m,m), A, Eigen::VectorXf::Ones(m);

    // keep track of basis for each player
    std::vector<std::vector<int>> bases(2);
    // P1 basis starts as s:{m+1..m+n}
    // P2 basis starts as r:{1..m}
    bases[0].resize(n);
    std::iota(bases[0].begin(), bases[0].end(),m);
    bases[1].resize(m);
    std::iota(bases[1].begin(), bases[1].end(),0);

    // start with pivot_var
    bool player = 0;
    const int initial_pivot_var = pivot_var; // LH ends when pivot varible is the same as begining
    int row_min_ratio;
    Eigen::VectorXf min_ratio_vec;

    const int max_iter = 500;
    for (int iter = 0; iter < max_iter; iter++)
    {
      // if not multithreaded, not check has_been_written
      // else if multithreaded, if another thread has found a soln, exit
      if (has_been_written != nullptr && has_been_written->load())
      {
        return false;
      }

      // min ratio rule means next pivot variable is the row with highest coefficient
      min_ratio_vec = tableaux[player].col(pivot_var).array()
        / tableaux[player].col(m+n).array();

      min_ratio_vec.maxCoeff(&row_min_ratio);

      // pivot, bringing pivot_var into the basis
      pivot(tableaux[player], row_min_ratio, pivot_var);

      // perform pivot 
      std::swap(pivot_var,bases[player][row_min_ratio]);

      // switch player
      player = !player;

      // check if complete
      if (pivot_var == initial_pivot_var)
      {
        // calculate mixed nash equilibria
        Eigen::VectorXd p1_strat = Eigen::VectorXd::Zero(m);
        Eigen::VectorXd p2_strat = Eigen::VectorXd::Zero(n);

        //player 1
        for(int idx=0; idx<n; idx++)// go over rows
        {
          if(bases[0][idx]<n)  // variable is in the basis
          {
            p1_strat[bases[0][idx]] = tableaux[0](idx,m+n)/tableaux[0](idx,bases[0][idx]);
          }
        }
        //player 2
        for(int idx=0; idx<m; idx++)// go over rows
        {
          if(bases[1][idx]>=m)  // variable is in the basis
          {
            p2_strat[bases[1][idx]-m] = tableaux[1](idx,m+n)/tableaux[1](idx,bases[1][idx]);
          }
        }

        // normalize probabilities
        p1_strat = p1_strat/p1_strat.sum();
        p2_strat = p2_strat/p2_strat.sum();
        
        // if not multithreaded, write soln
        if (has_been_written == nullptr)
        {
          std::cout << "has_been_written is null\n";
          mixed_strategy.push_back(p1_strat);
          mixed_strategy.push_back(p2_strat);
          return 1;
        }

        // else first check if another thread has found a soln
        //  since this loop started
        bool expected = false;
        bool exchanged = has_been_written->compare_exchange_strong(expected,true);
        // if successfully exchanged, this thread is the first to find a soln 
        //  and can safely write
        if (exchanged)
        {
          mixed_strategy.push_back(p1_strat);
          mixed_strategy.push_back(p2_strat);
          return 1;
        }
        // if not successful, this thread was beaten
        else
        {
          return 0;
        }
      }
    }
    std::cerr << "nash: Warning, mixedNashEquilibrium failed to find a mixed strategy solution within the max pivot limit\n";
    return 0;
  }

  bool pns(std::vector<Eigen::VectorXd>& mixed_strategy,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& B )
  {
    const int M = A.rows();
    const int N = A.cols();
    const int max_supp_size = std::min(M,N);
    std::vector<std::vector<int>> supports_p1;
    std::vector<int> actions_p1(M);
    std::iota(actions_p1.begin(),actions_p1.end(), 0);
    std::vector<std::vector<int>> supports_p2;
    std::vector<int> actions_p2(N);
    std::iota(actions_p2.begin(),actions_p2.end(), 0);

    // iterate through supports  by order of size
    // assumes that pure strats are already looked for
    for (int sup_size = 2; sup_size < max_supp_size; sup_size++)
    {
      // enumerate supports for player 1
      supports_p1 = nChoosek(M, sup_size);
      for (std::vector<int>& support_p1 : supports_p1)
      {
        bool support_1_valid = true;
        // remove conditionally dominated actions in actions_p2
        // use iterators from the end to not invalidate iterator
        for (auto a_p2 = actions_p2.end()-1; a_p2 != actions_p2.begin()-1; --a_p2)
        {
          for (auto a_p2_prime = actions_p2.begin(); a_p2_prime != actions_p2.end(); ++a_p2_prime)
          {
            // compare to all other player 2 actions
            bool is_cond_dom = true;
            if (a_p2_prime == a_p2)
            {continue;}
            // compare over all of player 1 support 
            for (int& a_p1 : support_p1)
            { // if any payoff for a_p2 is greater than or equal to a_p2_prime,
              // a_p2 is not conditionally dominated by a_p2_prime
              if (B(a_p1,*a_p2) >= B(a_p1,*a_p2_prime))
              {
                is_cond_dom = false;
                break;
              }
            }
            if (is_cond_dom)
            { // if conditionally dominated, remove from set of possible
              // actions and go to next a_p2
              actions_p2.erase(a_p2);
              break;
            }
          }
        }
        if (actions_p2.size() < sup_size)
        { continue; }
        else 
        {
          // check if any P1 is cond dom by all of remaining P2 actions
          for (auto a_p1 = support_p1.begin(); a_p1 != support_p1.end(); ++a_p1)
          {
            for (auto a_p1_prime = a_p1+1; a_p1_prime != support_p1.end(); ++a_p1_prime)
            {
              bool is_cond_dom = true;
              //compare over all of player 2 remaining actions
              for (const auto& a_p2:actions_p2)
              {
                if ( A(*a_p1,a_p2) >= A(*a_p1_prime,a_p2) )
                {
                  is_cond_dom = false;
                  break;
                }
              }
              if (is_cond_dom)
              {
                support_1_valid = false;
                break;
              }

            }
            if (!support_1_valid)
              break;
          }

          if(support_1_valid)
          {
            // loop over all combinations of remaining possible supports
            supports_p2 = nChoosek(actions_p2, sup_size);
            for (std::vector<int>& support_p2 : supports_p2)
            {
              bool these_supports_valid = true;
              // check if action_p1 is cond dom by the particular support_p2
              for (auto a_p1 = support_p1.begin(); a_p1 != support_p1.end(); ++a_p1)
              {
                for (auto a_p1_prime = a_p1+1; a_p1_prime != support_p1.end(); ++a_p1_prime)
                {
                  bool is_cond_dom = true;
                  //compare over all of player 2 remaining actions
                  for (const auto& a_p2:support_p2)
                  {
                    if ( A(*a_p1,a_p2) >= A(*a_p1_prime,a_p2) )
                    {
                      is_cond_dom = false;
                      break;
                    }
                  }
                  if (is_cond_dom)
                  {
                    these_supports_valid = false;
                    break;
                  }

                }
                if (!these_supports_valid)
                  break;
                else
                {
                  // feasibility function over both supports
                  if (feasibility_function(
                        mixed_strategy,
                        A, B, 
                        support_p1,
                        support_p2 )  )
                  {
                    return true;
                  }

                }

              }

            }
          }
        }


        // reset actions_p2
        actions_p2.resize(N);
        std::iota(actions_p2.begin(),actions_p2.end(), 0);

      }

    }

    return false;
  }


  int riskDominantEquilibrium(const std::vector<std::vector<int>>& equilibria,
      const Eigen::MatrixXd& A, 
      const Eigen::MatrixXd& B)
  {
    std::vector<int> h_eq = equilibria[0];
    int i_risk_dom = 0;
    for (int i = 0; i < equilibria.size(); i++)
    {
      std::vector<int> g_eq = equilibria[i];
      if ( ( A(h_eq[0],g_eq[1])-A(g_eq[0],g_eq[1]) ) *
          ( B(g_eq[0],h_eq[1])-B(g_eq[0],g_eq[1]) )    >

          ( A(g_eq[0],h_eq[1])-A(h_eq[0],h_eq[1]) ) *
          ( B(h_eq[0],g_eq[1])-B(h_eq[0],h_eq[1]) )    )
      { //equilibrium at position G risk dominates H
        h_eq = g_eq;
        i_risk_dom = i;
      }
    }
    return i_risk_dom;
  }

}
