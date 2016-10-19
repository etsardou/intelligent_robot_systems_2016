#ifndef OGMPP_METRICS_NODE_DEF
#define OGMPP_METRICS_NODE_DEF

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ogmpp_metrics
{

  class OgmppMetrics
  {
    public:
      OgmppMetrics(void);

      static float calculateLength(const nav_msgs::Path &path);
  };

}

#endif

