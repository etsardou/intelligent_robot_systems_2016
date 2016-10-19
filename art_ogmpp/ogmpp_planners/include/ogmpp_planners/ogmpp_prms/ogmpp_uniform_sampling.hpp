#ifndef OGMPP_UNIFORM_SAMPLING_DEF
#define OGMPP_UNIFORM_SAMPLING_DEF

#include "ogmpp_planners/ogmpp_abstract_planner.hpp"

namespace ogmpp_planners
{
  namespace prms
  {

    /**
     * @class UniformSampling
     * Inherits publicly from OgmppAbstractPlanner
     */
    class UniformSampling: public OgmppAbstractPlanner
    {
      private:

        /**
         * @brief Creates the uniform sampling graph
         * @param map [ogmpp_map_loader&] The map
         * @param begin [ogmpp_graph::Cell] The starting cell
         * @param end [ogmpp_graph::Cell] The ending cell
         */
        ogmpp_graph::Graph _createGraph(
            ogmpp_map_loader::Map &map, 
            ogmpp_graph::Cell begin, 
            ogmpp_graph::Cell end,
            std::map<std::string, double> parameters);

        std::map<std::string,double> _fixParameters(
          std::map<std::string, double> parameters);

    };

  }
}

#endif
