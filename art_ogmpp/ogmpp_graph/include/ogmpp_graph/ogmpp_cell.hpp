#ifndef OGMPP_CELL_DEF
#define OGMPP_CELL_DEF

#include <iostream>
#include <vector>
#include <map>
#include <cmath>

#include <ros/ros.h>

/**
 * @brief The OGMPP Graph related namespace
 */
namespace ogmpp_graph
{

  /**
   * @class Cell
   * @brief Handles an OGM cell
   */
  class Cell
  {
    public:
      /**< The cell's x coordinate */
      long x;
      /**< The cell's y coordinate */
      long y;

      /**
       * @brief Default constructor
       */
      Cell(void);
    
      /**
       * @brief Initializes a cell with its coordinates
       * @param x [long] The x coordinate
       * @param y [long] The y coordinate
       */
      Cell(long x, long y);

      /**
       * @brief Calculates the distance between two cells
       * @param cell [const Cell&] The other cell
       * @return float: The eucledian distance between the two cells
       */
      float distanceFrom(const Cell& cell);

      /**
       * @brief Calculates the square distance between two cells
       * @param cell [const Cell&] The other cell
       * @return float: The square of the eucledian distance between the two cells
       */
      float sqDistanceFrom(const Cell& cell);

      /**
       * @brief Prints the Cell's coordinates
       */
      void print(void);

      /**
       * @brief Overloading of == operator between two cells
       */
      bool operator==(const Cell& c);

  };

}

#endif
