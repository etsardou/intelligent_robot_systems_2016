#ifndef OGMPP_PLANNERS_NODE_DEF
#define OGMPP_PLANNERS_NODE_DEF

#include "ogmpp_planners/ogmpp_planner_factory.hpp"

#include "ogmpp_communications/OgmppPathPlanningMsg.h"
#include "ogmpp_communications/OgmppPathPlanningSrv.h"

#include "ogmpp_metrics/ogmpp_metrics_node.hpp"

#include "nav_msgs/Path.h"

/**< The generic ogmpp planners namespace */
namespace ogmpp_planners
{
  class OgmppPlanners
  {
    private:
      
      ros::NodeHandle _nh;

      ros::ServiceServer _server_path_planning;

      ros::Publisher _path_publisher;

      /**< Holds the map of the environment */
      ogmpp_map_loader::Map _map;

      OgmppPlannerFactory _planner_factory; 

      bool planCallback(
        ogmpp_communications::OgmppPathPlanningSrv::Request& req,
        ogmpp_communications::OgmppPathPlanningSrv::Response& res);

    public:
      OgmppPlanners(void);
  };

}



#endif

