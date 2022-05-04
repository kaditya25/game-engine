#include <queue>
#include <iostream>
#include "a_star3d.h"
#include <algorithm>
namespace game_engine
{
  // Anonymous namespace. Put any file-local functions or variables in here
  namespace
  {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node, a cost to reach that
    // node, and a heuristic cost from the current node to the destination.
    struct NodeWrapper
    {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node3D> node_ptr;

      // True cost to this node
      double cost;

      // Heuristic to end node
      double heuristic;

      // Equality operator
      bool operator==(const NodeWrapper &other) const
      {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    // Helper function. Compares the values of two NodeWrapper pointers.
    // Necessary for the priority queue.
    bool NodeWrapperPtrCompare(
        const std::shared_ptr<NodeWrapper> &lhs,
        const std::shared_ptr<NodeWrapper> &rhs)
    {
      return lhs->cost + lhs->heuristic > rhs->cost + rhs->heuristic;
    }

    ///////////////////////////////////////////////////////////////////
    // EXAMPLE HEURISTIC FUNCTION
    // YOU WILL NEED TO MODIFY THIS OR WRITE YOUR OWN FUNCTION
    ///////////////////////////////////////////////////////////////////
    double Heuristic(
        const std::shared_ptr<Node3D> &current_ptr,
        const std::shared_ptr<Node3D> &end_ptr)
    {
      double x1 = current_ptr->Data()[0];
      double y1 = current_ptr->Data()[1];
      double z1 = current_ptr->Data()[2];
      double x2 = end_ptr->Data()[0];
      double y2 = end_ptr->Data()[1];
      double z2 = end_ptr->Data()[2];

      return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) +  pow(z1 - z2, 2));

    }

    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;
    bool is_present(NodeWrapperPtr nwp, std::vector<NodeWrapperPtr> nwpVec)
    {
      for(int j=0;j<nwpVec.size();j++)
      {
        if( *(nwp) == *(nwpVec[j]) )
          return true;
      }
      return false;
    }

  }

  PathInfo AStar3D::Run(
      const Graph3D& graph,
      const std::shared_ptr<Node3D> start_ptr,
      const std::shared_ptr<Node3D> end_ptr)
  {
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
            const NodeWrapperPtr &,
            const NodeWrapperPtr &)>>
        to_explore(NodeWrapperPtrCompare);

    std::vector<NodeWrapperPtr> explored;

    ///////////////////////////////////////////////////////////////////
    // YOUR WORK GOES HERE
    PathInfo path_info;
    path_info.details.num_nodes_explored = 0;
    path_info.details.path_length = 1;
    path_info.details.path_cost = 0;
    path_info.details.run_time = timer.Stop();
    path_info.path = {};

    NodeWrapperPtr st_ptr = std::make_shared<NodeWrapper>();
    st_ptr->parent = nullptr;
    st_ptr->node_ptr = start_ptr;
    st_ptr->cost = 0;
    st_ptr->heuristic = Heuristic(start_ptr, end_ptr);
    to_explore.push(st_ptr);

     while (!to_explore.empty())
    {
      NodeWrapperPtr node_to_explore = to_explore.top();
      to_explore.pop();
      if (is_present(node_to_explore,explored))
      {
        continue;
      }

      if (*(node_to_explore->node_ptr) == *(end_ptr))
      {
        path_info.details.path_cost = node_to_explore->cost;
        path_info.path.push_back(node_to_explore->node_ptr);
        while (node_to_explore->parent != nullptr)
        {
          path_info.details.path_length += 1;  
          node_to_explore = node_to_explore->parent;
          path_info.path.push_back(node_to_explore->node_ptr);
        }
        std::reverse(path_info.path.begin(),path_info.path.end());
        path_info.details.run_time = timer.Stop();
        return path_info;
      }
      else
      {
        explored.push_back(node_to_explore);
        path_info.details.num_nodes_explored += 1;
        std::vector<DirectedEdge3D> edges = graph.Edges(node_to_explore->node_ptr);
        for (auto edge : edges)
        {
          NodeWrapperPtr neighbor = std::make_shared<NodeWrapper>();
          neighbor->parent = node_to_explore;
          neighbor->node_ptr = edge.Sink();
          neighbor->cost = node_to_explore->cost + edge.Cost();
          neighbor->heuristic = Heuristic(neighbor->node_ptr, end_ptr);
          to_explore.push(neighbor);
        }
      }
    }
  }
}
