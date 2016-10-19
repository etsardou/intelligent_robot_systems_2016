#include "ogmpp_graph.hpp"

namespace ogmpp_graph
{

  /**
   * @brief Default constructor
   */
  Graph::Graph(float map_resolution):
    _resolution(map_resolution)
  {
    ros::NodeHandle nh;
    _visualization_enabled = false;
    _visualization_delay_ms = 0; 
    _node_visualization_size = 0;
    _connections_visualization_size = 0;
    int rviz_delay_sec = 0;

    if(nh.hasParam("enable_visualization"))
    {
      nh.getParam("enable_visualization", _visualization_enabled);
    }
    if(nh.hasParam("visualization_delay_ms"))
    {
      nh.getParam("visualization_delay_ms", _visualization_delay_ms);
    }
    if(nh.hasParam("nodes_visualization_size"))
    {
      nh.getParam("nodes_visualization_size", _node_visualization_size);
    }
    if(nh.hasParam("connections_visualization_size"))
    {
      nh.getParam("connections_visualization_size", 
        _connections_visualization_size);
    }
    // Implement the visualization publisher in case we want to show the graph
    // on demand
    _visualization_pub = nh.advertise<visualization_msgs::Marker>(
      "visualization_marker", 0);
  }

  /**
   * @brief Initializes the Graph with a root node
   * @param cell [Cell] The coordinates to create the first node
   */
  Graph::Graph(Cell cell, float map_resolution):
    _resolution(map_resolution)
  {
    Node *n = new Node(cell);
    _nodes.insert( std::pair<unsigned long, Node*>(n->getId(), n) );

    if(_visualization_enabled) visualize();
  }

  /** 
   * @brief Prints information about the whole graph
   */
  void Graph::print(void)
  {
    for ( nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      ROS_INFO_STREAM( "------------" << std::endl );
      it->second->print();
    } 
  }

  /**
   * @brief Deallocates the allocated nodes and clears the graph
   */
  void Graph::clean(void)
  {
    for (nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      delete it->second;
    }
    _nodes.clear();
  }

  /**
   * @brief Removes a cell from the graph. It also removes it from all its 
   * neighbors
   * @param cell [Cell] The cell to be removed
   */
  void Graph::removeNode(Cell cell)
  {
    for (nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      it->second->removeNeighbor(cell);
    }
    _nodes.erase(Node::createCantorPairing(cell));

    if(_visualization_enabled) visualize();
  }

  /**
   * @brief Adds a node in the graph without assigning neighbors
   * @param c [Cell] The new node's coordinates
   * @return Node * : The new node
   */
  Node* Graph::addNode(Cell cell)
  {
    Node *n = new Node(cell);
    _nodes.insert( std::pair<unsigned long, Node*>(n->getId(), n) );

    if(_visualization_enabled) visualize();
    return n;
  }

  /**
   * @brief Creates a neighboring relation between two cells. If one of them
   * or both do not exist as nodes, it creates them
   * @param cp [Cell] The first cell
   * @param cq [Cell] The second cell
   * @param first_is_parent [bool] True if the first is the parent of the second
   */
  void Graph::makeNeighbor(Cell cp, Cell cq, bool first_is_parent, 
    float weight, bool reverse_neighborhood)
  {
    Node *np, *nq;
    // Check if cp exists
    if (_nodes.find(Node::createCantorPairing(cp)) == _nodes.end())
    {
      np = addNode(cp);
    }
    else
    {
      np = _nodes[Node::createCantorPairing(cp)];
    }

    // Check if cp exists
    if (_nodes.find(Node::createCantorPairing(cp)) == _nodes.end())
    {
      nq = addNode(cp);
    }
    else
    {
      nq = _nodes[Node::createCantorPairing(cq)];
    }

    nq->addNeighbor(np, first_is_parent, weight, reverse_neighborhood);

    if(_visualization_enabled) visualize();
  }

  /**
   * @brief Returns a node from a specific cell (if the node exists)
   * @param cell [Cell] The node's cell
   * @return Node* : The node corresponding to the specific cell
   */
  Node* Graph::getNode(Cell cell)
  {
    unsigned long id = Node::createCantorPairing(cell);
    if (_nodes.find(id) == _nodes.end())
    {
      return NULL;
    }
    return _nodes[id];
  }

  /**
   * @brief Returns all nodes of the graph
   * @return std::map<unsigned long, Node*> : All nodes
   */
  std::map<unsigned long, Node*> Graph::getNodes(void)
  {
    return _nodes;
  }

  /**
   * @brief Function to visualize the graph in rviz
   */
  void Graph::visualize(Cell begin, Cell end)
  {
    usleep(_visualization_delay_ms * 1000);

    // Visualize the nodes
    visualization_msgs::Marker m;

    m.header.frame_id = "map";
    m.header.stamp = ros::Time();
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.id = 0;
    m.ns = "ogmpp_graph_nodes";
    m.scale.x = _node_visualization_size;
    m.scale.y = _node_visualization_size;
    m.scale.z = _node_visualization_size;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;

    for(nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      geometry_msgs::Point p;
      p.x = it->second->getPose().x * _resolution;
      p.y = it->second->getPose().y * _resolution;

      m.points.push_back(p);
    }

    _visualization_pub.publish(m);

    // Visualize the connections
    visualization_msgs::Marker c;
    c.header.frame_id = "map";
    c.header.stamp = ros::Time();
    c.type = visualization_msgs::Marker::LINE_LIST;
    c.action = visualization_msgs::Marker::ADD;
    c.id = 1;
    c.ns = "ogmpp_graph_connections";
    c.scale.x = _connections_visualization_size;
    c.color.a = 1.0;
    c.color.r = 0.0;
    c.color.g = 1.0;
    c.color.b = 0.0;

    for(nodes_it it = _nodes.begin() ; it != _nodes.end() ; it++)
    {
      std::map<unsigned long, Node*> nmap = it->second->getNeighbors();
      for(nodes_it itn = nmap.begin() ; itn != nmap.end() ; itn++)
      {
        // Do not create double connections
        if(it->first > itn->first)
          continue;

        geometry_msgs::Point p1, p2;
        p1.x = it->second->getPose().x * _resolution;
        p1.y = it->second->getPose().y * _resolution;
        p2.x = itn->second->getPose().x * _resolution;
        p2.y = itn->second->getPose().y * _resolution;
        c.points.push_back(p1);
        c.points.push_back(p2);
      }
    }
    _visualization_pub.publish(c);

    if(begin.x == -1 || begin.y == -1)
    {
      return;
    }

    // Visualize the begin and end
    m.header.frame_id = "map";
    m.header.stamp = ros::Time();
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.id = 2;
    m.ns = "ogmpp_graph_nodes_begin_end";
    m.scale.x = _node_visualization_size * 2; // NOTE: These in params?
    m.scale.y = _node_visualization_size * 2;
    m.scale.z = _node_visualization_size * 2;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.points.clear();
    geometry_msgs::Point p;
    p.x = begin.x * _resolution;
    p.y = begin.y * _resolution;
    m.points.push_back(p);
    p.x = end.x * _resolution;
    p.y = end.y * _resolution;
    m.points.push_back(p);
    _visualization_pub.publish(m);

  }
}
