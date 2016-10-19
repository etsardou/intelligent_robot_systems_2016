#include <gtest/gtest.h>

#include <ogmpp_graph/ogmpp_graph.hpp>

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

TEST_F(OgmppUnitTest, default_creation)
{
  ogmpp_graph::Graph g;
  unsigned int n = g.getNodes().size();
  EXPECT_EQ(n, 0);
}

TEST_F(OgmppUnitTest, default_with_cell)
{
  ogmpp_graph::Cell c(10, 10);
  ogmpp_graph::Graph g(c);
  unsigned int n = g.getNodes().size();
  EXPECT_EQ(n, 1);
  ogmpp_graph::Node *node = g.getNode(c);
  EXPECT_EQ(node->getId(), ogmpp_graph::Node::createCantorPairing(c));
  EXPECT_EQ(node->getNeighbors().size(), 0);
}

TEST_F(OgmppUnitTest, add_neighbors)
{
  ogmpp_graph::Cell c(10, 10);
  ogmpp_graph::Graph g(c);
  g.addNode(ogmpp_graph::Cell(5, 5));
  EXPECT_EQ(g.getNodes().size(), 2);
  g.makeNeighbor(c, ogmpp_graph::Cell(5, 5));
  EXPECT_EQ(g.getNodes().size(), 2);
  EXPECT_EQ(g.getNode(c)->getNeighbors().size(), 1);
  EXPECT_EQ(g.getNode(ogmpp_graph::Cell(5, 5))->getNeighbors().size(), 1);
  bool b = g.getNode(ogmpp_graph::Cell(5, 5))->getNeighbor(c) == NULL;
  b = g.getNode(ogmpp_graph::Cell(10, 10))->getNeighbor(
    ogmpp_graph::Cell(5, 5)) == NULL;
  EXPECT_EQ(b, false);
}

TEST_F(OgmppUnitTest, remove_node)
{
  ogmpp_graph::Cell c(10, 10);
  ogmpp_graph::Graph g(c);
  g.addNode(ogmpp_graph::Cell(5, 5));
  g.makeNeighbor(c, ogmpp_graph::Cell(5, 5));
  g.removeNode(c);
  EXPECT_EQ(g.getNodes().size(), 1);
  ogmpp_graph::Node *nr = g.getNode(ogmpp_graph::Cell(5, 5));
  bool b = nr->getPose() == ogmpp_graph::Cell(5, 5);
  EXPECT_EQ(b, true);
  EXPECT_EQ(nr->getNeighbors().size(), 0);
}

TEST_F(OgmppUnitTest, clear_graph)
{
  ogmpp_graph::Cell c(10, 10);
  ogmpp_graph::Graph g(c);
  g.addNode(ogmpp_graph::Cell(5, 5));
  g.makeNeighbor(c, ogmpp_graph::Cell(5, 5));
  g.clean();
  EXPECT_EQ(g.getNodes().size(), 0);
}

/**
 * @brief The main function. Initialized the unit tests
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
