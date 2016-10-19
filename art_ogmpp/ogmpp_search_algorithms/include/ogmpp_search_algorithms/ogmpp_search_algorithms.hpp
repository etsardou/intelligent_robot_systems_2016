#ifndef OGMPP_SEARCH_ALGORITHMS_DEF
#define OGMPP_SEARCH_ALGORITHMS_DEF

#include "ogmpp_search_algorithms/ogmpp_a_star.hpp"

namespace ogmpp_search_algorithms
{

  class SearchAlgorithms
  {
    private:
      static AStar _a_star;

    public:
      static std::vector<ogmpp_graph::Cell> aStarSearch(
        ogmpp_graph::Graph &g,
        ogmpp_graph::Cell start,
        ogmpp_graph::Cell end);
  };
}

#endif
