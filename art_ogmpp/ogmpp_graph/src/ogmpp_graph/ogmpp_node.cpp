#include "ogmpp_node.hpp"

namespace ogmpp_graph
{

  /**
   * @brief Creates a unique id from two single values. It will be used
   * for easily searching the neighbors
   * @param cell [Cell] The cell whose coordinates will produce the Cantor
   * pairing value
   * @return unsinged long : The unique value
   */
  unsigned long Node::createCantorPairing(Cell cell)
  {
    // Added 0.5 for rounding reasons
    // Added 1 to consider id = 0 uninitialized
    return 0.5*(cell.x + cell.y)*(cell.x + cell.y + 1) + cell.y + 0.5 + 1;
  }

  /**
   * @brief Default constructor
   */
  Node::Node(void)
  {
    // Uninitialized id
    _id = 0;
    // Uninitialized parent id
    _parent_id = 0;
  }

  /**
   * @brief Initializes a node with a pose
   * @param pose [Cell] The node's coordinates
   */
  Node::Node(Cell pose, unsigned long parent_id)
  {
    this->_parent_id = parent_id;
    this->_pose = pose;
    // id is now initialized
    this->_id = Node::createCantorPairing(pose);
  }

  /**
   * @brief Returns the node's id
   * @return unsigned long : The node's id
   */
  unsigned long Node::getId(void)
  {
    return _id;
  }

  /**
   * @brief Returns the node's parent id
   * @return unsigned long : The node's parent id
   */
  unsigned long Node::getParent(void)
  {
    return _parent_id;
  }


  /**
   * @brief Returns the node's pose
   * @return Cell : The node's pose
   */
  Cell Node::getPose(void)
  {
    return _pose;
  }

  /**
   * @brief Prints information about the node
   */
  void Node::print(void)
  {
    ROS_INFO_STREAM ( "ID = " << _id );
    ROS_INFO_STREAM ( "Pose = [" << _pose.x << "," << _pose.y << "]");
    ROS_INFO_STREAM( "Neighbors:" );
    for (neigh_it it = _neighbors.begin() ; it != _neighbors.end() ; it++)
    {
      ROS_INFO_STREAM 
        ( "\t" << it->first << " [" << 
          it->second->getPose().x << "," << it->second->getPose().y 
          << "] w = " << _weights[it->first]);
    }
  }

  /**
   * @brief Adds a neighbor giving a node
   * @param node [Node*] The neighboring node
   * @param is_parent [bool] True if it is its parent
   * @param weight [float] Optional parameter for user-defined weights. If 
   * the value is -1 the Eucledian distance will be used
   * @param reverse_neighborhood [bool] If true the neighborhood is created 
   * both ways
   */
  void Node::addNeighbor(
    Node *node, 
    bool is_parent, 
    float weight, 
    bool reverse_neighborhood)
  {
    // Check if already exists
    if (_neighbors.find(node->getId()) != _neighbors.end())
    {
      return;
    }
    // Add the node here as neighbor
    _neighbors.insert( std::pair<unsigned long, Node*>(node->getId(), node) );
    if (is_parent)
    {
      _parent_id = node->getId();
    }
    float w = weight;
    if (w < 0)
    {
      w = this->_pose.distanceFrom(node->getPose());
    }
    _weights.insert( std::pair<unsigned long, float>(node->getId(), w) );

    // Add the reverse neighborhood
    if (reverse_neighborhood)
    {
      node->addNeighbor(this, is_parent, w, false);
    }
  }

  /**
   * @brief Removes a node from the neighbors list
   * @param node [Node*] The node to be erased
   */
  void Node::removeNeighbor(Node *node)
  {
    _neighbors.erase(node->getId());
    _weights.erase(node->getId());
  }

  /**
   * @brief Removes a node from the neighbors list
   * @param cell [Cell] The cell to be erased
   */
  void Node::removeNeighbor(Cell cell)
  {
    _neighbors.erase(Node::createCantorPairing(cell));
    _weights.erase(Node::createCantorPairing(cell));
  }

  /**
   * @brief Returns the neighbors  of the node
   * @return std::map<unsigned long, Node*> : The nodes
   */
  std::map<unsigned long, Node*> Node::getNeighbors(void)
  {
    return _neighbors;
  }

  /** 
   * @brief Returns the weight of a connection
   * @param id [unsigned long] The id of the other node
   * @return float : The weight
   */
  float Node::getWeight(unsigned long id)
  {
    if (_neighbors.find(id) == _neighbors.end())
    {
      return -1;
    }
    return _weights[id];
  }

  /** 
   * @brief Returns the weight of a connection
   * @param node [Node*] The other node
   * @return float : The weight
   */
  float Node::getWeight(Node* node)
  {
    unsigned long id = node->getId();
    if (_neighbors.find(id) == _neighbors.end())
    {
      return -1;
    }
    return _weights[id];
  }

  /** 
   * @brief Returns the weight of a connection
   * @param cell [Cell] The other node's cell
   * @return float : The weight
   */
  float Node::getWeight(Cell cell)
  {
    unsigned long id = Node::createCantorPairing(cell);
    if (_neighbors.find(id) == _neighbors.end())
    {
      return -1;
    }
    return _weights[id];
  }


  /**
   * @brief Sets the weight of a connection
   * @param node [Node*] The other node
   * @param w [float] The weight
   */
  void Node::setWeight(Node *node, float w)
  {
    unsigned long id = node->getId();
    if (_neighbors.find(id) == _neighbors.end())
    {
      return;
    }
    _weights[id] = w;
  }

  /**
   * @brief Sets the weight of a connection
   * @param cell [Cell] The other node's cell
   * @param w [float] The weight
   */
  void Node::setWeight(Cell cell, float w)
  {
    unsigned long id = Node::createCantorPairing(cell);
    if (_neighbors.find(id) == _neighbors.end())
    {
      return;
    }
    _weights[id] = w;
  }

  /**
   * @brief Sets the weight of a connection
   * @param id [unsigned long] The other node's id
   * @param w [float] The weight
   */
  void Node::setWeight(unsigned long id, float w)
  {
    if (_neighbors.find(id) == _neighbors.end())
    {
      return;
    }
    _weights[id] = w;
  }

  /**
   * @brief Returns a specific neighbor of the node
   * @param cell [Cell] The neighbor's cell
   * @return Node* : The requested node
   */
  Node* Node::getNeighbor(Cell cell)
  {
    unsigned long id = Node::createCantorPairing(cell);
    if (_neighbors.find(id) == _neighbors.end())
    {
      return NULL;
    }
    return _neighbors[id];
  }

}
