#ifndef OGMPP_NODE_DEF
#define OGMPP_NODE_DEF

#include "ogmpp_cell.hpp"

/**
 * @brief The OGMPP Graph related namespace
 */
namespace ogmpp_graph
{

  /**
   * @class Node
   * @brief Handles a node, along with its connections
   * Each node's id is derived from it's coordinates
   */
  class Node
  {
    
    // Typedef for easier iteration of the map
    typedef std::map<unsigned long, Node*>::iterator neigh_it;
    
    private:

      /**< The node's id */
      unsigned long _id;
      /**< The node's neighbors */
      std::map<unsigned long, Node*> _neighbors;
      /**< The connections' weights */
      std::map<unsigned long, float> _weights;
      /**< The node's pose */
      Cell _pose;
      /**< The node who created the current one */
      unsigned long _parent_id;
     
    public:
      /**
       * @brief Default constructor
       */
      Node(void);

      /**
       * @brief Initializes a node with a pose
       * @param pose [Cell] The node's coordinates
       * @param parent_id [unsigned long] The parent's id. Default value is 0
       */
      Node(Cell pose, unsigned long parent_id = 0);

      /**
       * @brief Creates a unique id from two single values. It will be used
       * for easily searching the neighbors
       * @param cell [Cell] The cell whose coordinates will produce the Cantor
       * pairing value
       * @return unsinged long : The unique value
       */
      static unsigned long createCantorPairing(Cell cell);

      /**
       * @brief Returns the node's id
       * @return unsigned long : The node's id
       */
      unsigned long getId(void);

      /**
       * @brief Returns the node's parent id
       * @return unsigned long : The node's parent id
       */
      unsigned long getParent(void);

      /**
       * @brief Returns the node's pose
       * @return Cell : The node's pose
       */
      Cell getPose(void);

      /**
       * @brief Prints information about the node
       */
      void print(void);

      /**
       * @brief Adds a neighbor giving a node
       * @param node [Node*] The neighboring node
       * @param is_parent [bool] True if it is its parent
       * @param weight [float] Optional parameter for user-defined weights. If 
       * the value is -1 the Eucledian distance will be used
       * @param reverse_neighborhood [bool] If true the neighborhood is created 
       * both ways
       */
      void addNeighbor(
        Node *node, 
        bool is_parent = true, 
        float weight = -1,
        bool reverse_neighborhood = false);

      /**
       * @brief Removes a node from the neighbors list
       * @param node [Node*] The node to be erased
       */
      void removeNeighbor(Node *node);

      /**
       * @brief Removes a node from the neighbors list
       * @param cell [Cell] The cell to be erased
       */
      void removeNeighbor(Cell cell);

      /**
       * @brief Returns the neighbors  of the node
       * @return std::map<unsigned long, Node*> : The nodes
       */
      std::map<unsigned long, Node*> getNeighbors(void);

      /**
       * @brief Returns a specific neighbor of the node
       * @param cell [Cell] The neighbor's cell
       * @return Node* : The requested node
       */
      Node* getNeighbor(Cell cell);
      
      /** 
       * @brief Returns the weight of a connection
       * @param id [unsigned long] The id of the other node
       * @return float : The weight
       */
      float getWeight(unsigned long id);
 
      /** 
       * @brief Returns the weight of a connection
       * @param cell [Cell] The cell of the other node
       * @return float : The weight
       */
      float getWeight(Cell cell);


      /** 
       * @brief Returns the weight of a connection
       * @param node [Node*] The other node
       * @return float : The weight
       */
      float getWeight(Node* node);

      /**
       * @brief Sets the weight of a connection
       * @param node [Node*] The other node
       * @param w [float] The weight
       */
      void setWeight(Node *node, float w);

      /**
       * @brief Sets the weight of a connection
       * @param cell [Cell] The other node's cell
       * @param w [float] The weight
       */
      void setWeight(Cell cell, float w);

      /**
       * @brief Sets the weight of a connection
       * @param id [unsigned long] The other node's id
       * @param w [float] The weight
       */
      void setWeight(unsigned long id, float w);

  };

}

#endif
