#include "ogmpp_planners/ogmpp_abstract_planner.hpp"

namespace ogmpp_planners
{

  OgmppAbstractPlanner::~OgmppAbstractPlanner(void)
  {
  }

  /**
   * @brief Function that fixes the path from start to stop using A*. 
   * Declared virtual if anyone wants to call another search algorithm
   * @param g [ogmpp_graph::Graph&] The produced graph
   * @param begin [ogmpp_graph::Cell] The start cell
   * @param end [ogmpp_graph::Cell] The ending cell
   */
  std::vector<ogmpp_graph::Cell> OgmppAbstractPlanner::_fixPath(
    ogmpp_graph::Graph& g,
    ogmpp_graph::Cell begin,
    ogmpp_graph::Cell end)
  {
    // Find the path
    return ogmpp_search_algorithms::SearchAlgorithms::aStarSearch(
      g, 
      begin, 
      end);
  }


  /**
   * @brief Visualizes the graph and the path
   */
  void OgmppAbstractPlanner::_visualize(
    ogmpp_graph::Graph& g,
    ogmpp_graph::Cell begin,
    ogmpp_graph::Cell end)
  {
    // On demand visualize
    g.visualize(begin, end);
  }


  /**
   * @brief Function that acts as the classe's frontend. Includes
   * the full implementation
   * @param map [ogmpp_map_loader::Map&] The map
   * @param begin [ogmpp_graph::Cell] The start cell
   * @param end [ogmpp_graph::Cell] The ending cell
   */
  std::vector<ogmpp_graph::Cell> OgmppAbstractPlanner::createPath(
      ogmpp_map_loader::Map& map,
      ogmpp_graph::Cell begin, 
      ogmpp_graph::Cell end,
      std::map<std::string, double> parameters)
    {
      _g.clean();
      _g = _createGraph(map, begin, end, parameters);
      _visualize(_g, begin, end);
      return _fixPath(_g, begin, end);
    }
}
