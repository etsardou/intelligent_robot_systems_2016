#include <gtest/gtest.h>

#include <ogmpp_graph/ogmpp_node.hpp>

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

TEST_F(OgmppUnitTest, cantor_testing)
{
  unsigned long n;
  ogmpp_graph::Cell c(5, 10);
  n = ogmpp_graph::Node::createCantorPairing(c);
  EXPECT_EQ(n, 131);
  c.x = 10;
  n = ogmpp_graph::Node::createCantorPairing(c);
  EXPECT_EQ(n, 221);
}

TEST_F(OgmppUnitTest, simple_creation)
{
  ogmpp_graph::Cell c(5, 10);
  ogmpp_graph::Node n(c);
  EXPECT_EQ(n.getId(), 130 + 1);
  EXPECT_EQ(n.getPose().x, 5);
  EXPECT_EQ(n.getPose().y, 10);
}

TEST_F(OgmppUnitTest, add_neighbor_test_default)
{
  ogmpp_graph::Cell c1(10, 2);
  ogmpp_graph::Cell c2(2, 10);
  ogmpp_graph::Node n1(c1);
  ogmpp_graph::Node n2(c2);
  
  n1.addNeighbor(&n2);
  std::map<unsigned long, ogmpp_graph::Node*> m = n1.getNeighbors();
  EXPECT_EQ(m.size(), 1);

  n1.addNeighbor(&n2);
  m = n1.getNeighbors();
  EXPECT_EQ(m.size(), 1);

  m = n2.getNeighbors();
  EXPECT_EQ(m.size(), 0);

  // The parent was set. 89 is the id of n2
  EXPECT_EQ(n1.getParent(), 89);
}

TEST_F(OgmppUnitTest, add_neighbor_test_not_default)
{
  ogmpp_graph::Cell c1(10, 10);
  ogmpp_graph::Cell c2(14, 13);
  ogmpp_graph::Node n1(c1);
  ogmpp_graph::Node n2(c2);
  
  n1.addNeighbor(&n2, false, 45.5, true);
  std::map<unsigned long, ogmpp_graph::Node*> m = n1.getNeighbors();
  EXPECT_EQ(m.size(), 1);

  n1.addNeighbor(&n2);
  m = n1.getNeighbors();
  EXPECT_EQ(m.size(), 1);

  // Reverse neighborhood was declared true
  m = n2.getNeighbors();
  EXPECT_EQ(m.size(), 1);

  EXPECT_FLOAT_EQ(n1.getWeight(&n2), 45.5);
  EXPECT_FLOAT_EQ(n2.getWeight(&n1), 45.5);
  EXPECT_FLOAT_EQ(n2.getWeight(n1.getId()), 45.5);

  EXPECT_EQ(n1.getParent(), 0);
  EXPECT_EQ(n2.getParent(), 0);

  n1.setWeight(&n2, 60.1);
  EXPECT_FLOAT_EQ(n1.getWeight(&n2), 60.1);

  n1.setWeight(n2.getId(), 63.1);
  EXPECT_FLOAT_EQ(n1.getWeight(&n2), 63.1);
}

TEST_F(OgmppUnitTest, remove_neighbor)
{
  ogmpp_graph::Cell c1(10, 10);
  ogmpp_graph::Cell c2(14, 13);
  ogmpp_graph::Node n1(c1);
  ogmpp_graph::Node n2(c2);
  
  n1.addNeighbor(&n2, false, 45.5, true);
  std::map<unsigned long, ogmpp_graph::Node*> m = n1.getNeighbors();
  EXPECT_EQ(m.size(), 1);

  n1.removeNeighbor(&n2);
  m = n1.getNeighbors();
  EXPECT_EQ(m.size(), 0);

  n1.addNeighbor(&n2);
  n1.removeNeighbor(n2.getPose());
  m = n1.getNeighbors();
  EXPECT_EQ(m.size(), 0);
}


/**
 * @brief The main function. Initialized the unit tests
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
