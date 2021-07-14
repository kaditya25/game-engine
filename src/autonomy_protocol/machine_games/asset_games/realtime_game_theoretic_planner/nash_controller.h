#ifndef NASH_CONTROLLER_H
#define NASH_CONTROLLER_H
#include "controller.h"
#include "sync_future.h"
#include "future_combinator.h"
#include "cost_calculator.h"
#include "nash.h"
#include "helper.h"
#include "thread_pool.h"
#include <random>

using costFunction = std::function<void(double&,
                                        const Eigen::VectorXd&, // x_self
                                        const Eigen::VectorXd&, // u_self
                                        const Eigen::VectorXd&, // x_other
                                        const Eigen::VectorXd&  // u_other
                                        )>;

using dynFunction = std::function<void(Eigen::VectorXd&, // x_k+1
    const Eigen::VectorXd&, // x_k
    const Eigen::VectorXd&, // u
    const double&           // dt
    )>;
class nashController : public Controller
{
  public:
    Eigen::VectorXd& target_;

    nashController(
      std::vector<Eigen::VectorXd>& u_enum_self, 
      std::vector<Eigen::VectorXd>& u_enum_other, 
      int& K_i, const int& K_steps, const double& dt,
      Eigen::VectorXd& x_0_self,
      Eigen::VectorXd& x_0_other,
      dynFunction& fDynPtr_self,
      dynFunction& fDynPtr_other,
      costFunction& jCostPtr_self,
      costFunction& jCostPtr_other,
      double vel_max, double acc_max, Eigen::VectorXd& target,
      const int num_tasks_per_th = 20,
      const int num_threads_per_pool = 500  ):
      x_dim_(x_0_self.size()), 
      u_dim_(u_enum_self[0].size()),
      n_self_(u_enumerated_self_.size()),
      n_other_(u_enumerated_other_.size()),

      u_enumerated_self_(u_enum_self),
      u_enumerated_other_(u_enum_other),

      K_i_(K_i),K_steps_(K_steps),dt_(dt),
      x_0_self_(x_0_self), 
      x_0_other_(x_0_other), 

      fDynPtr_self_(fDynPtr_self),
      fDynPtr_other_(fDynPtr_other),
      jCostPtr_self_(jCostPtr_self),
      jCostPtr_other_(jCostPtr_other),

      combinator_tasks_per_thread_(num_tasks_per_th),

      pool_futures_(num_threads_per_pool),

      self_futures_( x_futures_self_,
                    u_futures_self_,
                    u_enumerated_self_,
                    K_i_, K_steps_,
                    dt_,
                    x_0_self_,
                    fDynPtr_self_,
                    pool_futures_, 
                    combinator_tasks_per_thread_ ),

      other_futures_( x_futures_other_,
                    u_futures_other_,
                    u_enumerated_other_,
                    K_i_, K_steps_,
                    dt_,
                    x_0_other_,
                    fDynPtr_other_,
                    pool_futures_,
                    combinator_tasks_per_thread_ ),

      vel_max_(vel_max),
      acc_max_(acc_max),
      target_(target),

      self_cost_calc_( cost_matrix_self_,
                        x_futures_self_,
                        u_futures_self_,
                        x_futures_other_,
                        u_futures_other_,
                        K_i_, K_steps_,
                        dt_, jCostPtr_self_,
                        vel_max_,acc_max_,target_
          ),

      other_cost_calc_( cost_matrix_other_,
                        x_futures_other_,
                        u_futures_other_,
                        x_futures_self_,
                        u_futures_self_,
                        K_i_, K_steps_,
                        dt_, jCostPtr_other_,
                        vel_max_,acc_max_,target_
          )

      {
        reserve_memory(); 
      }

    ~nashController()
    {
      pool_futures_.shutdown();
    }


    void reserve_memory();

    void start_calculations();

    void join_calculations();

    Eigen::VectorXd calcControlInput() override;

    void resetNextTimeStep() override;

    Eigen::MatrixXd value_matrix_self();
    Eigen::MatrixXd value_matrix_other();

  private:

    // given a Eigen::Vector representing a discrete probability distribution, 
    // return an index randomly drawn from the pdf
    int draw_from_distr(Eigen::VectorXd pdf);

    const int x_dim_, u_dim_;
    std::vector<syncFuture> x_futures_self_;
    std::vector<syncFuture> u_futures_self_;
    std::vector<syncFuture> x_futures_other_;
    std::vector<syncFuture> u_futures_other_;
    std::vector<Eigen::VectorXd> u_enumerated_self_;
    std::vector<Eigen::VectorXd> u_enumerated_other_;

    Eigen::VectorXd& x_0_self_;
    Eigen::VectorXd& x_0_other_;
    dynFunction& fDynPtr_self_;
    dynFunction& fDynPtr_other_;
    costFunction& jCostPtr_self_;
    costFunction& jCostPtr_other_;
    int& K_i_, K_steps_;
    const double& dt_;
    const int combinator_tasks_per_thread_;
    const int n_self_; 
    const int n_other_; 

    threadPool pool_futures_;

    futureCombinator self_futures_;
    futureCombinator other_futures_;

    Eigen::MatrixXd cost_matrix_self_;
    Eigen::MatrixXd cost_matrix_other_;

    costCalculator self_cost_calc_;
    costCalculator other_cost_calc_;

    double vel_max_;
    double acc_max_;
};

#endif
