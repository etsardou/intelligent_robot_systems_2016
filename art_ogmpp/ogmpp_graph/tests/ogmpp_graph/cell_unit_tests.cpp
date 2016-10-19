#include <gtest/gtest.h>

#include <ogmpp_graph/ogmpp_cell.hpp>

class OgmppUnitTest : public ::testing::Test
{
  protected:

    /**
     * @brief Default constructor
     */
    OgmppUnitTest()
    {
    }
    /**
     * @brief Sets up the class variables for each unit test call
     */
    virtual void SetUp()
    {
    }

    /**
     * @brief This function is called after the termination of each test. 
     * Destroys the dynamically alloced variables
     */
    virtual void TearDown()
    {
    }
};

TEST_F(OgmppUnitTest, simple_creation)
{
  ogmpp_graph::Cell c(5, 10);
  EXPECT_EQ(c.x, 5);
  EXPECT_EQ(c.y, 10);
}

TEST_F(OgmppUnitTest, check_distance)
{
  ogmpp_graph::Cell c1(5, 10);
  ogmpp_graph::Cell c2(5, 20);
  EXPECT_EQ(c1.distanceFrom(c2), 10);

  ogmpp_graph::Cell c3(5, 10);
  EXPECT_EQ(c1.distanceFrom(c3), 0);
}

TEST_F(OgmppUnitTest, check_sq_distance)
{
  ogmpp_graph::Cell c1(5, 10);
  ogmpp_graph::Cell c2(5, 20);
  EXPECT_EQ(c1.sqDistanceFrom(c2), 100);

  ogmpp_graph::Cell c3(5, 10);
  EXPECT_EQ(c1.sqDistanceFrom(c3), 0);
}

/**
 * @brief The main function. Initialized the unit tests
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
