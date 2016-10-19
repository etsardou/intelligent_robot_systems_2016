#include "ogmpp_search_algorithms/ogmpp_search_algorithms.hpp"

namespace ogmpp_search_algorithms
{

  AStar SearchAlgorithms::_a_star = AStar();

  std::vector<ogmpp_graph::Cell> SearchAlgorithms::aStarSearch(
    ogmpp_graph::Graph &g,
    ogmpp_graph::Cell start,
    ogmpp_graph::Cell end)
  {
    return _a_star.search(g, start, end);
  }

}
