#include "ogmpp_map_loader/ogmpp_map_loader.hpp"

namespace ogmpp_map_loader
{
  Map::Map(void):
    _map_initialized(false)
  {
    if(_nh.hasParam("map_topic"))
    {
      _nh.getParam("map_topic", _map_topic);
      _map_topic = "/" + _map_topic;
      ROS_INFO_STREAM("ogmpp_map_loader: Map topic: " << _map_topic); 
    }
    else
    {
      ROS_ERROR("ogmpp_map_loaded: No map topic defined. Assuming \"/map\"");
    }

    ros::spinOnce();
    _map_subscriber = _nh.subscribe(_map_topic.c_str(), 1, 
      &Map::mapCallback, this);
    ros::spinOnce();

    _set_map_service = _nh.advertiseService(
      "/ogmpp_path_planners/set_map",
      &Map::mapSetCallback, this);
  }

  bool Map::mapSetCallback(
    ogmpp_communications::OgmppSetMapSrv::Request& req,
    ogmpp_communications::OgmppSetMapSrv::Response& res)
  {
    setMap(req.map);
    return true;
  }

  void Map::setMap(const nav_msgs::OccupancyGrid& map)
  {
    // Free the previous map
    if(_map_initialized)
    {
      for(unsigned int i = 0 ; i < _ros_map.info.width ; i++)
      {
        delete [] _map[i];
        delete [] _brushfire[i];
      }
      delete [] _map;
      delete [] _brushfire;
    }

    _ros_map = map;

    _resolution = map.info.resolution;
    unsigned int width = map.info.width;
    unsigned int height = map.info.height;

    ROS_INFO_STREAM(width << " " << height);

    /**< NOTE: Must check if the map was allocated beforehand */

    _map = new char*[width];
    _brushfire = new long*[width];
    for(unsigned int i = 0 ; i < width ; i++)
    {
      _map[i] = new char[height];
      _brushfire[i] = new long[height];
      for(unsigned int j = 0 ; j < height ; j++)
      {
        _map[i][j] = map.data[j * width + i];

        if(_map[i][j] < 50)
          _brushfire[i][j] = -1;
        else
          _brushfire[i][j] = 0; 
      }
    }

    // Create brushfire (Manhattan distance transformation)
    bool changed = true;
    long counter = 0;
    while(changed)
    {
      changed = false;
      for(unsigned int i = 1 ; i < width - 1 ; i++)
      {
        for(unsigned int j = 1 ; j < height - 1 ; j++)
        {
          // Implement 4 point brushfire
          if(_brushfire[i][j] == counter)
          {
            if(_brushfire[i][j - 1] == -1)
            {
              _brushfire[i][j - 1] = counter + 1;
              changed = true;
            }
            if(_brushfire[i - 1][j] == -1) 
            {
              _brushfire[i - 1][j] = counter + 1;
              changed = true;
            }
            if(_brushfire[i][j + 1] == -1) 
            {
              _brushfire[i][j + 1] = counter + 1;
              changed = true;
            }
            if(_brushfire[i + 1][j] == -1) 
            {
              _brushfire[i + 1][j] = counter + 1;
              changed = true;
            }
          }
        }
      }
      counter++;
    }

    ROS_INFO_STREAM("ogmpp_map_loader: Map initialized");
    _map_initialized = true;

  }

  void Map::mapCallback(const nav_msgs::OccupancyGrid& map)
  {
    setMap(map);
  }

  bool Map::isMapInitialized(void)
  {
    return _map_initialized;
  }

  float Map::getResolution(void)
  {
    return _resolution;
  }

  std::pair<unsigned int, unsigned int> Map::getMapSize()
  {
    return std::pair<unsigned int, unsigned int>
      (_ros_map.info.width, _ros_map.info.height);
  }

  bool Map::isOccupied(long x, long y)
  {
    if (x >= _ros_map.info.width  || x < 0 || 
      y >= _ros_map.info.height || y < 0)
    {
      return false;
    }
    return _map[x][y] > 50;
  }
  bool Map::isUnoccupied(long x, long y)
  {
    if (x >= _ros_map.info.width || x < 0 || 
      y >= _ros_map.info.height || y < 0)
    {
      return false;
    }
    return _map[x][y] < 50;
  }
  bool Map::isUnknown(long x, long y)
  {
    if (x >= _ros_map.info.width || x < 0 || 
      y >= _ros_map.info.height || y < 0)
    {
      return false;
    }
    return _map[x][y] == 50 || _map[x][y] == -1;
  }
  bool Map::isValid(long x, long y)
  {
    if (x >= _ros_map.info.width || x < 0 || 
      y >= _ros_map.info.height || y < 0)
    {
      return false;
    }
    return !isOccupied(x, y) && !isUnknown(x, y);
  }


  long Map::getDistanceTransformation(long x, long y)
  {
    if (x >= _ros_map.info.width || x < 0 || 
      y >= _ros_map.info.height || y < 0)
    {
      return -1;
    }
    return _brushfire[x][y];
  }

}

