#include <queue>

#include "dijkstra2d.h"

namespace game_engine {
  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node and a cost to reach
    // that node.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node2D> node_ptr;
      double cost;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

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

    // Helper function. Compares the values of two NodeWrapper pointers.
    // Necessary for the priority queue.
    bool NodeWrapperPtrCompare(
        const std::shared_ptr<NodeWrapper>& lhs, 
        const std::shared_ptr<NodeWrapper>& rhs) {
      return lhs->cost > rhs->cost;
    }
  }

  PathInfo Dijkstra2D::Run(
      const Graph2D& graph, 
      const std::shared_ptr<Node2D> start_ptr, 
      const std::shared_ptr<Node2D> end_ptr) {
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

        for(const DirectedEdge2D& edge: graph.Edges(node_to_explore->node_ptr)) 
        {
          NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
          nw_ptr->node_ptr = edge.Sink();
          nw_ptr->cost = node_to_explore->cost + edge.Cost();
          nw_ptr->parent = node_to_explore;          
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
