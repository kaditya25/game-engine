#include "sync_future.h"
#include "future_combinator.h"
#include "cost_calculator.h"
#include "nash.h"
#include "helper.h"
#include "nash_controller.h"
#include "vel_match_controller.h"
#include "thread_pool.h"
#include <random>
#include <math.h>
#include <map>
#include <functional>
#include <stdio.h>
#include <stdlib.h>

// pursuer is pursuer/defender
// evader is evader/targeter // 6/18 notes:
// multithreaded approach works...
//    condition variable seems to miss notifications sometimes? recalling notify once done reading seems to be an ok workaround
//    in cost calculator, a lost of time is wasted making the threads. This is probably a major loss of time overall, and should probably implement some sort of threadpooling to frontload time.
//    with n=9, K=4, t_realtime~=30sec, using 100% of 4 cores
//    with n=9, K=3, t_realtime~<=1sec

using costFunction = std::function<void(double&,
    const Eigen::VectorXd&, // x_pursuer
    const Eigen::VectorXd&, // u_pursuer
    const Eigen::VectorXd&, // x_evader
    const Eigen::VectorXd&  // u_evader
    )>;

using dynFunction = std::function<void(Eigen::VectorXd&, // x_k+1
    const Eigen::VectorXd&, // x_k
    const Eigen::VectorXd&, // u
    const double&           // dt
    )>;

// system dynamics are here
void fDynPursuer(Eigen::VectorXd& x_1, 
    const Eigen::VectorXd& x_0, 
    const Eigen::VectorXd& u, 
    const double& dt) 
{
  Eigen::MatrixXd A(4,4);
  A << Eigen::MatrixXd::Identity(2,2), Eigen::MatrixXd::Identity(2,2)*dt,
    Eigen::MatrixXd::Zero(2,2), Eigen::MatrixXd::Identity(2,2);
  Eigen::MatrixXd B(4,2);
  B << Eigen::MatrixXd::Identity(2,2)*0.5*std::pow(dt,2), Eigen::MatrixXd::Identity(2,2)*dt;
  x_1 = A*x_0 + B*u;
  return;
}

void fDynEvader(Eigen::VectorXd& x_1, 
    const Eigen::VectorXd& x_0, 
    const Eigen::VectorXd& u, 
    const double& dt) 
{
  Eigen::MatrixXd A(4,4);
  A << Eigen::MatrixXd::Identity(2,2), Eigen::MatrixXd::Identity(2,2)*dt,
    Eigen::MatrixXd::Zero(2,2), Eigen::MatrixXd::Identity(2,2);
  Eigen::MatrixXd B(4,2);
  B << Eigen::MatrixXd::Identity(2,2)*0.5*std::pow(dt,2), Eigen::MatrixXd::Identity(2,2)*dt;
  x_1 = A*x_0 + B*u;
  return;
}

// pursuer/defender cost function is here
void jCostPursuer( double& cost_total,
    const Eigen::VectorXd& x_self, 
    const Eigen::VectorXd& u_self, 
    const Eigen::VectorXd& x_other,
    const Eigen::VectorXd& u_other)
{
  const double acc_max = 0.4+.01;
  const double vel_max = 2+.01;

  Eigen::DiagonalMatrix<double,4> Q_pur;
  Q_pur.diagonal() << 5,5,3,3;
  Eigen::DiagonalMatrix<double,2> R_pur;
  R_pur.diagonal() << 1,1;
  Eigen::DiagonalMatrix<double,2> Q_target;
  Q_target.diagonal() << 5,5;
  Eigen::VectorXd target(2); target << 0,0;
  Eigen::VectorXd e = x_self-x_other;
  Eigen::VectorXd e_target = x_other.head(2)-target;

  // cumulative cost calculation
  cost_total += e.transpose()*Q_pur*e;
  cost_total +=  - e_target.transpose()*Q_target*e_target;

  // end condition: if target has been reached
  if (e_target.norm() < 2)
  { cost_total += 2000/ (e_target.norm()+.1); }
  // end condition: if evader has been caught
  if (e.head(2).norm() < 2)
  { cost_total += -1000/ (e.norm()+.1); }

  // soft limits on acceleration and velocity 
  if (u_self.norm()>acc_max)
    cost_total += 20000;
  if (x_self.tail(2).norm()>vel_max)
    cost_total += 20000;



  return;
}
// evader/attacker cost function is here
void jCostEvader( double& cost_total,
    const Eigen::VectorXd& x_self, 
    const Eigen::VectorXd& u_self, 
    const Eigen::VectorXd& x_other,
    const Eigen::VectorXd& u_other)
{
  const double acc_max = 0.4+.01;
  const double vel_max = 2+.01;

  Eigen::DiagonalMatrix<double,4> Q_pur;
  Q_pur.diagonal() << 5,5,0,0;
  Eigen::DiagonalMatrix<double,2> R_pur;
  R_pur.diagonal() << 1,1;
  Eigen::DiagonalMatrix<double,2> Q_target;
  Q_target.diagonal() << 5,5;
  //Q_target.diagonal() << 4,4;
  Eigen::VectorXd target(2); target << 0,0;
  Eigen::VectorXd e = x_self-x_other;
  Eigen::VectorXd e_target = x_self.head(2)-target;

  // cumulative cost calculation
  cost_total += -e.transpose()*Q_pur*e;
  cost_total += e_target.transpose()*Q_target*e_target;

  // end condition: if target has been reached
  //if (e_target.norm() < 2)
  //{ 
  //  cost_total += -2000/ (e_target.norm()+.1 );
  //  cost_total += -2000* ( x_self.tail<2>().norm() );
  //}
  //// end condition: if evader has been caughtk
  //if (e.norm() < 2)
  //{ cost_total += 1000/ (e.norm()+.1); }

  // soft limits on acceleration and velocity 
  if (u_self.norm()>acc_max)
    cost_total += 20000;
  if (x_self.tail(2).norm()>vel_max)
    cost_total += 20000;

  return;
}

int main(int argc, char* argv[]) {
  bool singleshot = false;
  bool verbose = false;
  double x_start, y_start;

  if (argc < 3)
  {
    std::cout << "no starting point given, running grid\n";
  }
  else 
  {
    x_start = strtod(argv[1], NULL);
    y_start = strtod(argv[2], NULL);
    singleshot = true;
    verbose = true;
  }

  int K_start = 1;
  const int K_steps = 1;
  int n = 8;
  const int K_steps_evader = 2;
  int n_evader = 8;
  // initial conditions for 'best' plot
  //const int K_steps = 2;
  //int n = 8;
  const double dt = 1;
  double time_sim = 0;
  // initial condition
  Eigen::VectorXd x_evader(4);
  x_evader << -10.0,0.0, 0.0, 0.0;
  Eigen::VectorXd x_pursuer(4);
  x_pursuer << -10,2,0,0;

  const double acc_max = 0.4;
  const double vel_max = 2;

  // enumerate possible control
  std::vector<Eigen::VectorXd> u_enumerated(n+1);
  for (int i=0; i < n; i++)
  {
    u_enumerated[i].resize(2);
    u_enumerated[i][0] = cos(2*M_PI*i/n) * acc_max;
    u_enumerated[i][1] = sin(2*M_PI*i/n)* acc_max;
  }
  u_enumerated[n] = Eigen::VectorXd::Zero(2);

  std::vector<Eigen::VectorXd> u_enum_evader(n_evader+1);
  for (int i=0; i < n_evader; i++)
  {
    u_enum_evader[i].resize(2);
    u_enum_evader[i][0] = cos(2*M_PI*i/n_evader) * acc_max;
    u_enum_evader[i][1] = sin(2*M_PI*i/n_evader)* acc_max;
  }
  
  std::vector<Eigen::VectorXd> u_enum_evader_pursuer(9);
  for (int i=0; i < 8; i++)
  {
    u_enum_evader_pursuer[i].resize(2);
    u_enum_evader_pursuer[i][0] = cos(2*M_PI*i/8) * acc_max;
    u_enum_evader_pursuer[i][1] = sin(2*M_PI*i/8)* acc_max;
  }
  u_enum_evader_pursuer[8] = Eigen::VectorXd::Zero(2);

  u_enum_evader[n_evader] = Eigen::VectorXd::Zero(2);
  //u_enum_evader = u_enumerated;


  costFunction costFuncPursuer = jCostPursuer;
  costFunction costFuncEvader = jCostEvader; 
  dynFunction dynFuncPursuer = fDynPursuer;
  dynFunction dynFuncEvader = fDynEvader;

  Eigen::VectorXd dumb(2);
  dumb << 0,0;

  nashController nash_pursuer(
      u_enumerated,
      u_enum_evader,
      K_start, K_steps, dt,
      x_pursuer, x_evader,
      dynFuncPursuer,dynFuncEvader,
      costFuncPursuer,costFuncEvader
      );


  velMatchController velmatch_pursuer(
      x_pursuer, x_evader, 
      dumb,
      acc_max, vel_max, dt);


  nashController evader(
      u_enum_evader,
      u_enum_evader_pursuer,
      K_start, K_steps_evader, dt,
      x_evader, x_pursuer,
      dynFuncEvader, dynFuncPursuer,
      costFuncEvader, costFuncPursuer
      );

  Eigen::VectorXd acc_pursuer, acc_evader;
  std::chrono::duration<double>  t_runtime;
  double t_sim_velmatch, t_sim_nash;
  int winner = 0; // -1 is evader, 1 is pursuer
  int i=0;

  // store results in map :/
  //std::map<std::tuple<double,double>,double> nash_time_map, velmatch_time_map;

  std::chrono::time_point<std::chrono::system_clock> t_start;
  std::chrono::time_point<std::chrono::system_clock> t_end;
  std::chrono::duration<double> diff ;

  auto EigVecToTuple = []( const Eigen::VectorXd& vec)
  { return std::make_tuple(vec[0],vec[1]); };

  auto LinSpaced = []( const double start, const double end, const double delta )
  { std::vector<double> ret((int) ((end-start)/delta)+1); ret[0] = start;
    for( auto r=ret.begin()+1; r!= ret.end(); ++r ) *r = *(r-1)+delta;
    return ret; };

  std::vector<double> x_vals = LinSpaced(-10,0,.25);
  std::vector<double> y_vals = LinSpaced(2,5,.25);

  Eigen::MatrixXd nash_times(x_vals.size(),y_vals.size());
  Eigen::MatrixXd velmatch_times(x_vals.size(),y_vals.size());

  double catch_dist = .25;
  double max_dist = 20;
  double target_dist = .5;
  double max_time = 40;


  if(singleshot)
  {
    std::chrono::duration<double> cumulative_runtime ;
    int num_runs=0;
        x_pursuer << x_start,y_start,0,0;
        x_evader << -10,0,0,0;
        // reset
        winner = 0;
        time_sim = 0;
        K_start = 1;
        std::cout << "pursuer starting at "; helper::print(x_pursuer.transpose());

        t_start = std::chrono::system_clock::now();
        while(true)
        {

          if ( (x_pursuer-x_evader).head(2).norm() < catch_dist )
          {
            winner = 1;
            std::cout << "pursuer got!\n";
            break;
          }
          else if ( (x_pursuer-x_evader).head(2).norm() > max_dist  )
          {
            winner = -1;
            std::cout << "evader evaded !\n";
            break;
          }
          else if ( (x_evader.head(2)).norm() < target_dist )
          {
            winner = -1;
            std::cout << "evader got to target!\n";
            //    helper::print(x_evader.head(2));
            break;
          }
          else if ( time_sim > max_time )
          {
            winner = 0;//consider a tie for purposes of comparison and plotting
            std::cout << "evader chased off!\n";
            break;
          }

        auto run_start = std::chrono::system_clock::now();
          acc_pursuer = nash_pursuer.calcControlInput();
          nash_pursuer.resetNextTimeStep();
        auto run_end = std::chrono::system_clock::now();
        cumulative_runtime += run_end-run_start; num_runs++;

          acc_evader = evader.calcControlInput();
          evader.resetNextTimeStep();
          K_start++;
          time_sim+= dt;

          if(verbose)
          {
            std::cout << "evader state is \n";
            helper::print(x_evader);
            std::cout << "evader accel is \n";
            helper::print(acc_evader);
            std::cout << "pursuer state is \n";
            helper::print(x_pursuer);
            std::cout << "pursuer accel is \n";
            helper::print(acc_pursuer);
            std::cout << "simulation time: "<<time_sim<<"\n";
          }
          i++;

          dynFuncEvader(x_evader,x_evader,acc_evader,dt);
          dynFuncPursuer(x_pursuer,x_pursuer,acc_pursuer,dt);

        }
        {
          if(verbose)
          {
            t_end = std::chrono::system_clock::now();
            diff = t_end-t_start;
            t_runtime += diff;
            std::cout << "nashgame realtime: "<<diff.count() << "\n";
          }
        }
        t_sim_nash = winner * time_sim;
        // reset
        winner = 0;
        time_sim = 0;
        K_start = 1;
        x_pursuer << x_start,y_start,0,0;
        x_evader << -10,0, 0, 0;

        t_start = std::chrono::system_clock::now();
        while(true)
        {

          if ( (x_pursuer-x_evader).head(2).norm() < catch_dist )
          {
            winner = 1;
            std::cout << "pursuer got!\n";
            break;
          }
          else if ( (x_pursuer-x_evader).head(2).norm() > max_dist  )
          {
            winner = -1;
            std::cout << "evader evaded !\n";
            break;
          }
          else if ( (x_evader.head(2)).norm() < target_dist )
          {
            winner = -1;
            std::cout << "evader got to target!\n";
            //    helper::print(x_evader.head(2));
            break;
          }
          else if ( time_sim > max_time )
          {
            winner = 0;//consider a tie for purposes of comparison and plotting
            std::cout << "evader chased off!\n";
            break;
          }

          acc_pursuer = velmatch_pursuer.calcControlInput();
          acc_evader = evader.calcControlInput();
          velmatch_pursuer.resetNextTimeStep();
          evader.resetNextTimeStep();
          K_start++;
          time_sim+= dt;
          i++;


          if(verbose)
          {
            std::cout << "evader state is \n";
            helper::print(x_evader);
            std::cout << "evader accel is \n";
            helper::print(acc_evader);
            std::cout << "pursuer state is \n";
            helper::print(x_pursuer);
            std::cout << "pursuer accel is \n";
            helper::print(acc_pursuer);
            std::cout << "simulation time: "<<time_sim<<"\n";
          }


          dynFuncEvader(x_evader,x_evader,acc_evader,dt);
          dynFuncPursuer(x_pursuer,x_pursuer,acc_pursuer,dt);
        }
        {
          if(verbose)
          {
            t_end = std::chrono::system_clock::now();
            diff = t_end-t_start;
            t_runtime += diff;
            std::cout << "velmatch realtime: "<<diff.count() << "\n";
          }
        }
        t_sim_velmatch = winner * time_sim;

        std::cout << "nash time is " << t_sim_nash << "\n";
        std::cout << "velmatch time is " << t_sim_velmatch << "\n";

            std::cout << "nash pursuer cumulative time is " << cumulative_runtime.count() << "\n";
            std::cout << "nash pursuer average runtime is " << cumulative_runtime.count()/num_runs << "\n";
  }

  if(!singleshot)
  {
    for (int x_i=0; x_i<x_vals.size(); x_i++)
      //for (int pursuer_x=p_x_min; pursuer_x<=p_x_max; pursuer_x++)
    {
      double pursuer_x = x_vals[x_i];
      for (int y_i=0; y_i<y_vals.size(); y_i++)
        //for (int pursuer_y=p_y_min; pursuer_y<=p_y_max; pursuer_y++)
      {
        double pursuer_y = y_vals[y_i];

        x_pursuer << pursuer_x,pursuer_y,0,0;
        x_evader << -10,0,0,0;
        // reset
        winner = 0;
        time_sim = 0;
        K_start = 1;
        std::cout << "pursuer starting at "; helper::print(x_pursuer.transpose());

        t_start = std::chrono::system_clock::now();
        while(true)
        {
          if ( (x_pursuer-x_evader).head(2).norm() < catch_dist )
          {
            winner = 1;
            std::cout << "pursuer got!\n";
            break;
          }
          else if ( (x_pursuer-x_evader).head(2).norm() > max_dist  )
          {
            winner = -1;
            std::cout << "evader evaded !\n";
            break;
          }
          else if ( (x_evader.head(2)).norm() < target_dist )
          {
            winner = -1;
            std::cout << "evader got to target!\n";
            //    helper::print(x_evader.head(2));
            break;
          }
          else if ( time_sim > max_time )
          {
            winner = 0;//consider a tie for purposes of comparison and plotting
            std::cout << "evader chased off!\n";
            break;
          }

          acc_pursuer = nash_pursuer.calcControlInput();
          acc_evader = evader.calcControlInput();
          nash_pursuer.resetNextTimeStep();
          evader.resetNextTimeStep();
          K_start++;
          time_sim+= dt;

          if(verbose)
          {
            std::cout << "evader state is \n";
            helper::print(x_evader);
            std::cout << "evader accel is \n";
            helper::print(acc_evader);
            std::cout << "pursuer state is \n";
            helper::print(x_pursuer);
            std::cout << "pursuer accel is \n";
            helper::print(acc_pursuer);
            std::cout << "simulation time: "<<time_sim<<"\n";
          }
          i++;

          dynFuncEvader(x_evader,x_evader,acc_evader,dt);
          dynFuncPursuer(x_pursuer,x_pursuer,acc_pursuer,dt);

        }
        {
          if(verbose)
          {
            t_end = std::chrono::system_clock::now();
            diff = t_end-t_start;
            t_runtime += diff;
            std::cout << "nashgame realtime: "<<diff.count() << "\n";
          }
        }
        t_sim_nash = winner * time_sim;
        std::cout << "nash ended\n";
        // reset
        winner = 0;
        time_sim = 0;
        K_start = 1;
        x_pursuer << pursuer_x,pursuer_y,0,0;
        x_evader << -10,0, 0, 0;

        t_start = std::chrono::system_clock::now();
        while(true)
        {

          if ( (x_pursuer-x_evader).head(2).norm() < catch_dist )
          {
            winner = 1;
            std::cout << "pursuer got!\n";
            break;
          }
          else if ( (x_pursuer-x_evader).head(2).norm() > max_dist  )
          {
            winner = -1;
            std::cout << "evader evaded !\n";
            break;
          }
          else if ( (x_evader.head(2)).norm() < target_dist )
          {
            winner = -1;
            std::cout << "evader got to target!\n";
            //    helper::print(x_evader.head(2));
            break;
          }
          else if ( time_sim > max_time )
          {
            winner = 0;//consider a tie for purposes of comparison and plotting
            std::cout << "evader chased off!\n";
            break;
          }

          acc_pursuer = velmatch_pursuer.calcControlInput();
          acc_evader = evader.calcControlInput();
          velmatch_pursuer.resetNextTimeStep();
          evader.resetNextTimeStep();
          K_start++;
          time_sim+= dt;
          i++;


          if(verbose)
          {
            std::cout << "evader state is \n";
            helper::print(x_evader);
            std::cout << "evader accel is \n";
            helper::print(acc_evader);
            std::cout << "pursuer state is \n";
            helper::print(x_pursuer);
            std::cout << "pursuer accel is \n";
            helper::print(acc_pursuer);
            std::cout << "simulation time: "<<time_sim<<"\n";
          }


          dynFuncEvader(x_evader,x_evader,acc_evader,dt);
          dynFuncPursuer(x_pursuer,x_pursuer,acc_pursuer,dt);
        }
        {
          if(verbose)
          {
            t_end = std::chrono::system_clock::now();
            diff = t_end-t_start;
            t_runtime += diff;
            std::cout << "velmatch realtime: "<<diff.count() << "\n";
          }
        }
        t_sim_velmatch = winner * time_sim;
        std::cout << "velmatch ended\n";

        std::cout << "nash time is " << t_sim_nash << "\n";
        std::cout << "velmatch time is " << t_sim_velmatch << "\n";
        nash_times(x_i,y_i) = t_sim_nash;
        velmatch_times(x_i,y_i) = t_sim_velmatch;
      }
    }
    helper::printToMatlab(nash_times,"nash_times");
    helper::printToMatlab(velmatch_times,"velmatch_times");
  }

  return 0;
}
