#include <queue>

#include "a_star3d.h"

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
    }

  }

  std::vector<std::shared_ptr<Node3D>> returnPath(std::shared_ptr<NodeWrapper> en){
    std::vector<std::shared_ptr<Node3D>> path = {en->node_ptr};
    while(en->parent!=nullptr){
      en = en->parent;
      path.push_back(en->node_ptr);
    }
    std::reverse(path.begin(),path.end());
    return path;
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

    // Create a NodeWrapperPtr
    NodeWrapperPtr s_ptr = std::make_shared<NodeWrapper>();
    s_ptr->parent = nullptr;
    s_ptr->node_ptr = start_ptr;
    s_ptr->cost = 0;
    s_ptr->heuristic = Heuristic(s_ptr->node_ptr, end_ptr);
    to_explore.push(s_ptr);

    ///////////////////////////////////////////////////////////////////
    NodeWrapperPtr nte;
    while(!to_explore.empty()){
      nte = to_explore.top();
      to_explore.pop();

      if(std::find_if(explored.begin(),explored.end(),[nte](const NodeWrapperPtr e){return *(nte)==*(e);})!=explored.end()){
        continue;
      }

      else if(*(nte->node_ptr)==*(end_ptr)){
        break;
      }

      else{
        explored.push_back(nte);

        const std::vector<DirectedEdge3D> edges = graph.Edges(nte->node_ptr);
        for(const DirectedEdge3D& edge: edges) {
          NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
          nw_ptr->parent = nte;
          nw_ptr->node_ptr = edge.Sink();
          nw_ptr->cost = nte->cost + edge.Cost();
          nw_ptr->heuristic = Heuristic(nw_ptr->node_ptr, end_ptr);
          to_explore.push(nw_ptr);
        }
      }
    }
    ///////////////////////////////////////////////////////////////////

    // Create a PathInfo
    PathInfo path_info;
    path_info.path = returnPath(nte);
    path_info.details.num_nodes_explored = explored.size();
    path_info.details.path_length = path_info.path.size();
    path_info.details.path_cost = nte->cost;
    path_info.details.run_time = timer.Stop();

    // You must return a PathInfo
    return path_info;
  }
  
}
