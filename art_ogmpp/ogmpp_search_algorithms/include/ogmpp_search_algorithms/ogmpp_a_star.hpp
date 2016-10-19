#ifndef OGMPP_A_STAR_DEF
#define OGMPP_A_STAR_DEF

#include "ogmpp_graph/ogmpp_graph.hpp"

namespace ogmpp_search_algorithms
{

  class AStar
  {
    private:

    public:

      AStar(void);

      static std::vector<ogmpp_graph::Cell> search(
        ogmpp_graph::Graph &g,
        ogmpp_graph::Cell start,
        ogmpp_graph::Cell end);
  };

}

#endif
