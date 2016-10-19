#include "ogmpp_search_algorithms/ogmpp_a_star.hpp"

namespace ogmpp_search_algorithms
{

  AStar::AStar(void)
  {

  }

  std::vector<ogmpp_graph::Cell> AStar::search(
    ogmpp_graph::Graph &g,
    ogmpp_graph::Cell start,
    ogmpp_graph::Cell end)
  {

    // Typedefs for iterators to ease our lives
    typedef std::set<ogmpp_graph::Node*>::iterator nodes_set_it;
    typedef std::map<unsigned long, ogmpp_graph::Node*>::iterator nodes_map_it;

    std::vector<ogmpp_graph::Cell> ret;

    std::set<ogmpp_graph::Node*> closed_set;
    std::set<ogmpp_graph::Node*> open_set;

    ogmpp_graph::Node *start_node;
    ogmpp_graph::Node *end_node;

    // Sanity checks
    if(g.getNode(start) == NULL)
    {
      return ret;
    }
    start_node = g.getNode(start);
    open_set.insert(start_node);

    if(g.getNode(end) == NULL)
    {
      return ret;
    }
    end_node = g.getNode(end);

    std::map<ogmpp_graph::Node*, ogmpp_graph::Node*> came_from;
    came_from.insert(
      std::pair<ogmpp_graph::Node*, ogmpp_graph::Node*>(start_node, NULL));

    // Initialize cost maps
    std::map<ogmpp_graph::Node*, float> g_score;

    g_score.insert( std::pair<ogmpp_graph::Node*, float>(start_node, 0) );
    
    std::map<ogmpp_graph::Node*, float> f_score;
    f_score.insert( std::pair<ogmpp_graph::Node*, float>(start_node,
        g_score[start_node] + start.distanceFrom(end) ) );

    // Start the main loop
    while( open_set.size() != 0)
    {
      // Find element in open set with minimum f_score
      ogmpp_graph::Node* min_key = NULL;
      float min_val = std::numeric_limits<float>::infinity();

      for(nodes_set_it it = open_set.begin() ; it != open_set.end() ; it++)
      {
        if(f_score[*it] < min_val)
        {
          min_key = *it;
          min_val = f_score[*it];
        }
      }

      // Our current is ...
      ogmpp_graph::Node *current = min_key;
      // Check if the goal is reached
      if(current == end_node)
      {
        ogmpp_graph::Node *runner = came_from[end_node];
        ret.push_back(end_node->getPose());
        while(runner != NULL)
        {
          ret.push_back(runner->getPose());
          runner = came_from[runner];
        }
        return ret;
      }

      open_set.erase(current);
      closed_set.insert(current);

      // Check the current's neighbors
      std::map<unsigned long, ogmpp_graph::Node*> neigh = 
        current->getNeighbors();

      for(nodes_map_it it = neigh.begin() ; it != neigh.end() ; it++)
      {
        ogmpp_graph::Node *neighbor = it->second;
        // The neighbor is already evaluated?
        if( closed_set.find(it->second) != closed_set.end() )
          continue;

        // Check the length of this path
        float tentative_g_score = g_score[current] +
          current->getPose().distanceFrom(neighbor->getPose());

        // Check if we have a new node
        if( open_set.find(neighbor) == open_set.end() )
          open_set.insert(neighbor);
        else if(tentative_g_score >= g_score[neighbor])
          continue;   // This is not a better path

        // Update costs and values
        if(came_from.find(neighbor) == came_from.end())
          came_from.insert(
            std::pair<ogmpp_graph::Node*, ogmpp_graph::Node*>(
              neighbor, current) );
        else
          came_from[neighbor] = current;

        if(g_score.find(neighbor) == g_score.end())
          g_score.insert(std::pair<ogmpp_graph::Node*, float>(
              neighbor, tentative_g_score));
        else
          g_score[neighbor] = tentative_g_score;

        float neigh_f_sc = g_score[neighbor] +
          neighbor->getPose().distanceFrom(end); // Dist from neigh to goal

        if(f_score.find(neighbor) == f_score.end())
          f_score.insert(std::pair<ogmpp_graph::Node*, float>(
              neighbor, neigh_f_sc) );
        else
          f_score[neighbor] = neigh_f_sc;
      }
    }

    return ret;
  }

}
