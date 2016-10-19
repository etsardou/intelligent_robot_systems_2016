#include "ogmpp_planners/ogmpp_planner_factory.hpp"

#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"

namespace ogmpp_planners
{
  OgmppAbstractPlanner* OgmppPlannerFactory::getPlanner(std::string type)
  {
    
    if(type == "uniform_prm") return new prms::UniformSampling();
    return NULL;
  }

}
