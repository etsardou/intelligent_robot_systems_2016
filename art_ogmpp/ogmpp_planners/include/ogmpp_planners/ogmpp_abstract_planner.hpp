#ifndef OGMPP_PLANNER_ABSTRACT_DEF
#define OGMPP_PLANNER_ABSTRACT_DEF

#include "ogmpp_graph/ogmpp_graph.hpp"
#include "ogmpp_map_loader/ogmpp_map_loader.hpp"
#include "ogmpp_search_algorithms/ogmpp_search_algorithms.hpp"

/**< The generic ogmpp planners namespace */
namespace ogmpp_planners
{

  /**
   * @class OgmppAbstractPlanner
   * @brief Implements the abstract functionalities of a graph-based planner
   * Each planner has the following functionalities:
   * - Create a graph which contains the start and target
   * - Fix a path on that graph from the start to the target
   * - Visualize if requested
   */
  class OgmppAbstractPlanner
  {
    private:

      // TODO: Erase this and add a visualizer
      ogmpp_graph::Graph _g;

      /**
       * @brief Function that creates the graph structure of the planner. It
       * is pure virtual, thus any class inheriting it must implement it
       * @param map [ogmpp_map_loader::Map&] The map
       * @param begin [ogmpp_graph::Cell] The start cell
       * @param end [ogmpp_graph::Cell] The ending cell
       */
      virtual ogmpp_graph::Graph _createGraph(
        ogmpp_map_loader::Map &map,
        ogmpp_graph::Cell begin,
        ogmpp_graph::Cell end,
        std::map<std::string, double> parameters) = 0;

      virtual std::map<std::string,double> _fixParameters(
        std::map<std::string, double> parameters) = 0;

      /**
       * @brief Function that fixes the path from start to stop using A*. 
       * Declared virtual if anyone wants to call another search algorithm
       * @param g [ogmpp_graph::Graph&] The produced graph
       * @param begin [ogmpp_graph::Cell] The start cell
       * @param end [ogmpp_graph::Cell] The ending cell
       */
      virtual std::vector<ogmpp_graph::Cell> _fixPath(
        ogmpp_graph::Graph& g,
        ogmpp_graph::Cell begin,
        ogmpp_graph::Cell end);

      /**
       * @brief Visualizes the graph and the path
       */
      void _visualize(
        ogmpp_graph::Graph& g,
        ogmpp_graph::Cell begin,
        ogmpp_graph::Cell end);

    public:

      /**< The ROS NodeHandle, needed to load params */
      ros::NodeHandle _nh;

      virtual ~OgmppAbstractPlanner(void);

      /**
       * @brief Function that acts as the classe's frontend. Includes
       * the full implementation
       * @param map [ogmpp_map_loader::Map&] The map
       * @param begin [ogmpp_graph::Cell] The start cell
       * @param end [ogmpp_graph::Cell] The ending cell
       */
      std::vector<ogmpp_graph::Cell>  
        createPath(
          ogmpp_map_loader::Map &map, 
          ogmpp_graph::Cell begin, 
          ogmpp_graph::Cell end,
          std::map<std::string, double> parameters);
  };

}



#endif

