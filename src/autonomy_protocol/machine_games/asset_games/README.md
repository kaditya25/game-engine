# Asset Games

Implementation of [Nick Montalbano's quasi-nash optimal discrete controller][1] within the game-engine framework. Partially a re-implementation of his Matlab simulation with the following changes:

* Written in C++
* Multi-threaded propagation of system dynamics
* Multi-threaded calculation of the normal form game
* Multi-threaded algorithm to find mixed Nash equilibria
* Velocity-matching controller that respects velocity and acceleration limits


## Dependencies

`Eigen`

`Threads`


## Top Level Design

This section explains the higher-level framework of the asset-games library and the role of each portion of the source code.

The asset-games library is designed to with two main ideas: generality and speed. Though the particular implementation is used for point mass quads, the framework should be applicable to more complex dynamics and cost functions. 

Both velocity matching (VM) and quasi-nash optimal (QNO) controllers are derived from the abstract Controller class, which simply contains the abstract `Controller::calcControlInput()` and `Controller::resetNextTimeStep()` functions. By using a container of Controller pointers, a mix of both can be called. This also gives a simple way of comparing these controllers to others in the future. 

Controllers are assumed to be in the scope of a simulator that stores and updates states, simulation time, and enumerated controls. Controllers simply return a calculated control input for that single time step, and the simulator is expected to forward propagate until the next time step. 

Mathematical vectors and matrices are almost all represented by `Eigen::VectorXd` and `Eigen::MatrixXd` for extendability, while `std::vector` is used for containers. This means that fixed size matrices are not supported, even though they could (would) be faster.  


### Structure

Asset-games library's primary user-interface is the abstract `Controller` class and its two virtual functions. Currently implemented `Controller`s are just the `nashController` and `velMatchController`. 

`velMatchController` is an implementation of the VM controller. It provides an example of a self contained controller, which uses no other source files and operates purely on the current states of the game. This VM implementation is designed to time optimally intercept a target moving with known constant velocity within a velocity and acceleration magnitude limit.

`nashController` is the implementation of the QNO controller and was the main focus of the summer. The QNO controller is expected to find the control output for a player consistent with a (risk-dominant) nash optimal equilibrium of the finite time horizon discretized global game. 
A `nashController` contains two pairs of `futureCombinator` and `costCalculator` classes, one for the player and the other for the opponent. It also maintains the `threadPool` for the `futureCombinator`, and the various supporting data structures. 

`futureCombinator` calculates the permutations of enumerated control inputs and forward propagates the state based on the given dynamics model. Dynamics simulation is multi-threaded via the thread pool, and these expected future states and controls are stored within the thread-safe `syncFuture` class.

`costCalculator` uses a given cost function model and calculates the future cost based on the set of control input permutations and associated state propagations. This is multi-threaded, but not via a thread pool because of thread sync issues. Costs are written directly to a cost matrix passed by reference, but since each thread has a defined portion of the matrix, no locks are necessary for writing to the matrix. 

Both use `syncFuture` to provide thread-safe, efficient, one write / many read access to states and control inputs. Each `syncFuture` takes advantage of a `std::shared_timed_mutex` and `std::condition_variable_any` for locking and notification. 

`nashController` calculates nash equilibria via functions in the `nash` namespace. `nash` contains a naive method to find pure nash equilibria, if they exist, and a couple different ways to calculate mixed nash equilibria. There is an attempt to implement the sorted support enumeration algorithm of Porter, Nudelman, & Shoham, but it runs much slower than Lemke-Howson for large games. `nash::mixedNashEquilibrium` is a simple multi-threaded implementation of the Lemke-Howson algorithm and is the main interface. 

In addition, there are supporting classes and declarations used throughout that may be useful for other projects. The `helper` namespace contains functions for pretty printing `Eigen::MatrixXd` and options to output formatted for pasting into Matlab. `threadPool` provides a true implementation of a C++ thread pool, reducing the (probably negligible) overhead of creating and destroying threads.


## Implementation

This section goes into the details of the C++ implementation, what steps were taken to speed up calculations, and anything to keep in mind going forward.

Where applicable, the library is written with considerations for multi-threading and possibilities for GPU calculations through CUDA. The problem is massively parallelizable, and taking advantage of a GPU could give orders of magnitude more performance. 


### Memory

Variables are passed by reference as much as possible within `nashController` to reduce expensive copying. If using the `nashController` front-end, memory management should be managed during construction. If not, variables passed by reference to `costCalculator` and `futureCombinator` are expected to be pre-initialized and not go out of scope during the simulation. References also allow other classes to update member variables easily.

### Multi-Threading

Multi-threading is implemented in the `futureCombinator`, `costCalculator`, and `nash::mixedNashEquiliibrium`. `futureCombinator` and `costCalculator` each contain independent threads, and communication only needs to happen from `futureCombinator` threads to `costCalculator` threads.

Thread notification is done through a shared mutex and associated condition variable via `std::shared_timed_mutex` and `std::condition_variable_any` (C++14). A shared mutex can have two different types of locks: a write lock (`std::unique_lock`) and a read lock (`std::shared_lock`). Only a single thread can hold the write lock, but multiple threads can simultaneously hold the read lock provided there is no write lock. Rather than trying to lock repeatedly in a loop, the condition variable lets a thread sleep until notified of an update. `std::condition_variable_any` is required as `std::condition_variable` only works with standard mutexes.

A Mixed nash equilibrium is calculated via a simple parallel implementation of the Lemke-Howson algorithm. Each thread is started with a different initial pivot variable, and the first to return is the resulting mixed equilibrium. Since very little needs to be passed between threads, each thread simply exits when a `std::atomic<bool>` is updated. Concurrency is achieved with careful management of the atomic boolean and associated  return memory. 

### Extendability

Rather than using fixed definitions of cost and dynamics functions, `nashController` uses functor aliases for both. Functors let these functions be either static or member functions (wrapped with `std::bind` or lambdas) that could have considerations for other states beyond the templated ones. Note that functors and their arguments are also passed by reference, as copying of functors with complex states is slow.

[1]: https://gitlab.com/radionavlab/papers/pursuit-evasion-stochastic
