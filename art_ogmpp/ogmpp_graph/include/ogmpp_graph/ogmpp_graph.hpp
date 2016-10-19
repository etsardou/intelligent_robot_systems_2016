#ifndef OGMPP_GRAPH_DEF
#define OGMPP_GRAPH_DEF

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "ogmpp_node.hpp"

namespace ogmpp_graph
{
  /**
   * @class Graph
   * @brief Handles a Graph which contains nodes
   */
  class Graph
  {

    // Typedef for easier iteration of the map
    typedef std::map<unsigned long, Node*>::iterator nodes_it;

    private:
      /**< The graph's nodes */
      std::map<unsigned long, Node*> _nodes;

      /**< Flag to enable visualization */
      bool _visualization_enabled;

      /**< Holds the ms delay between each visualization */
      int _visualization_delay_ms;

      /**< The visualization publisher */
      ros::Publisher _visualization_pub;

      /**< The map's resolution */
      float _resolution;
    
      /**< The node's visualization size */
      double _node_visualization_size;

      /**< The connection's visualization size */
      double _connections_visualization_size;

    public:
      /**
       * @brief Default constructor
       * @param map_resolution [float] The map's resolution needed for the 
       * visualization
       */
      Graph(float map_resolution = 0.1);

      /**
       * @brief Initializes the Graph with a root node
       * @param cell [Cell] The coordinates to create the first node
       * @param map_resolution [float] The map's resolution needed for the 
       * visualization
       */
      Graph(Cell cell, float map_resolution = 0.1);

      /** 
       * @brief Prints information about the whole graph
       */
      void print(void);

      /**
       * @brief Deallocates the allocated nodes and clears the graph
       */
      void clean(void);

      /**
       * @brief Creates a neighboring relation between two cells. If one of them
       * or both do not exist as nodes, it creates them
       * @param cp [Cell] The first cell
       * @param cq [Cell] The second cell
       * @param first_is_parent [bool] True if the first is the parent of the second
       * @param weight [float] The connection's weight. -1 if the euclidean distance
       * is used
       * @param reverse_neighborhood [bool] True for creating the connection
       * both ways
       */
      void makeNeighbor(Cell cp, Cell cq, bool first_is_parent = false,
        float weight = -1, bool reverse_neighborhood = true);
      
      /**
       * @brief Adds a node in the graph without assigning neighbors
       * @param c [Cell] The new node's coordinates
       * @return Node * : The new node
       */
      Node* addNode(Cell cell);
      
      /**
       * @brief Removes a cell from the graph. It also removes it from all its 
       * neighbors
       * @param cell [Cell] The cell to be removed
       */
      void removeNode(Cell cell);

      /**
       * @brief Returns a node from a specific cell (if the node exists)
       * @param cell [Cell] The node's cell
       * @return Node* : The node corresponding to the specific cell
       */
      Node* getNode(Cell cell);
      
      /**
       * @brief Returns all nodes of the graph
       * @return std::map<unsigned long, Node*> : All nodes
       */
      std::map<unsigned long, Node*> getNodes(void);

      /**
       * @brief Function to visualize the graph in rviz
       */
      void visualize(Cell begin = Cell(), Cell end = Cell());
  };

}

#endif
