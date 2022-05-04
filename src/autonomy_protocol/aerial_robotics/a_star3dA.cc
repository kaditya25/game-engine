#include <queue>

#include "a_star3dA.h"

namespace game_engine {
  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node, a cost to reach that
    // node, and a heuristic cost from the current node to the destination.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node3D> node_ptr;

      // True cost to this node
      double cost;

      // Heuristic to end node
      double heuristic;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    // Helper function. Compares the values of two NodeWrapper pointers.
    // Necessary for the priority queue.
    bool NodeWrapperPtrCompare(
        const std::shared_ptr<NodeWrapper>& lhs, 
        const std::shared_ptr<NodeWrapper>& rhs) {
      return lhs->cost + lhs->heuristic > rhs->cost + rhs->heuristic;
    }

    ///////////////////////////////////////////////////////////////////
    // EXAMPLE HEURISTIC FUNCTION
    // YOU WILL NEED TO MODIFY THIS OR WRITE YOUR OWN FUNCTION
    ///////////////////////////////////////////////////////////////////
    double Heuristic(
        const std::shared_ptr<Node3D>& current_ptr,
        const std::shared_ptr<Node3D>& end_ptr) {
          return std::sqrt(std::pow(current_ptr->Data().x()-end_ptr->Data().x(),2) +
          std::pow(current_ptr->Data().y()-end_ptr->Data().y(),2) + 
          std::pow(current_ptr->Data().z()-end_ptr->Data().z(),2));
          
      // return 0; //zero heuristic
      // return (current_ptr->Data().x() - end_ptr->Data().x()) + (current_ptr->Data().y() - end_ptr->Data().y()); //Taking the Manhattan distane (Overestimating the cost)  
      // return sqrt((current_ptr->Data() - end_ptr->Data()).transpose() * ( current_ptr->Data() - end_ptr->Data())); //Taking the l2 norm error (Underestimating the cost)
    }

  }

  using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;
  bool subset(const std::vector<NodeWrapperPtr> &explored_nodes, const NodeWrapperPtr current_node)
  {
    for(const std::shared_ptr<NodeWrapper> explored_node: explored_nodes) 
    {
      if ( *(explored_node) == *(current_node) )
      {
        return true;
      }
    }
    return false;
  }

  PathInfo AStar3D::Run(
      const Graph3D& graph, 
      const std::shared_ptr<Node3D> start_ptr, 
      const std::shared_ptr<Node3D> end_ptr) {
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

    ///////////////////////////////////////////////////////////////////
    // SETUP
    // DO NOT MODIFY THIS
    ///////////////////////////////////////////////////////////////////
    Timer timer;
    timer.Start();

    // Use these data structures
    std::priority_queue<
      NodeWrapperPtr,
      std::vector<NodeWrapperPtr>,
      std::function<bool(
          const NodeWrapperPtr&, 
          const NodeWrapperPtr& )>> 
        to_explore(NodeWrapperPtrCompare);

    std::vector<NodeWrapperPtr> explored;

    ///////////////////////////////////////////////////////////////////
    // YOUR WORK GOES HERE
    // SOME EXAMPLE CODE INCLUDED BELOW
    ///////////////////////////////////////////////////////////////////

    // Create a NodeWrapperPtr
    NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
    nw_ptr->parent = nullptr;
    nw_ptr->node_ptr = start_ptr;
    nw_ptr->cost = 0;
    nw_ptr->heuristic = Heuristic(start_ptr, end_ptr);
    to_explore.push(nw_ptr);

    NodeWrapperPtr node_to_explore;
    while (!to_explore.empty())
    {
      node_to_explore = to_explore.top();
      to_explore.pop();

      if (subset(explored,node_to_explore))
      {
        continue;
      }
      if (*(node_to_explore->node_ptr) == *end_ptr)
      {
        break;
      }
      else
      {
        explored.push_back(node_to_explore);

        // Iterate through the list of edges and add the neighbors
        for(const DirectedEdge3D& edge: graph.Edges(node_to_explore->node_ptr)) 
        {
          NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
          nw_ptr->cost = node_to_explore->cost + edge.Cost();
          nw_ptr->heuristic = Heuristic(edge.Sink(), end_ptr);
          nw_ptr->parent = node_to_explore;
          nw_ptr->node_ptr = edge.Sink();          
          to_explore.push(nw_ptr);
        }
      }
    }

    NodeWrapperPtr node = node_to_explore;

    // Create a PathInfo
    PathInfo path_info;
    path_info.details.num_nodes_explored = explored.size()+1;
    path_info.details.path_cost = node->cost;

    path_info.details.path_length = 1;
    while( node->parent != nullptr )
    {
      path_info.path.push_back( node->node_ptr );
      node = node->parent; 
      path_info.details.path_length++;
    }
    path_info.path.push_back( node->node_ptr );
    std::reverse( path_info.path.begin(), path_info.path.end() );

    path_info.details.run_time = timer.Stop();
    
    return path_info;
  }
  
}
