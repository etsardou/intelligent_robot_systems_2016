#include "ogmpp_metrics/ogmpp_metrics_node.hpp"

namespace ogmpp_metrics
{
  OgmppMetrics::OgmppMetrics(void)
  {

  }

  float OgmppMetrics::calculateLength(const nav_msgs::Path &path)
  {
    if(path.poses.size() <= 1)
    {
      return 0;
    }

    float length = 0;
    for(unsigned int i = 0 ; i < path.poses.size() - 1 ; i++)
    {
      length += sqrt(
        pow(path.poses[i].pose.position.x - 
          path.poses[i + 1].pose.position.x, 2.0) +
        pow(path.poses[i].pose.position.y - 
          path.poses[i + 1].pose.position.y, 2.0)
        );
    }
    return length;
  }

}

