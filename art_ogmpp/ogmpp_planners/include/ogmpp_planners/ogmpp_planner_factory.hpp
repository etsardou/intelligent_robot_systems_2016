#ifndef OGMPP_PLANNER_FACTORY_DEF
#define OGMPP_PLANNER_FACTORY_DEF

#include "ogmpp_planners/ogmpp_abstract_planner.hpp"

/**< The generic ogmpp planners namespace */
namespace ogmpp_planners
{
  class OgmppPlannerFactory
  {
    public:
      OgmppAbstractPlanner* getPlanner(std::string type);
  };

}



#endif

