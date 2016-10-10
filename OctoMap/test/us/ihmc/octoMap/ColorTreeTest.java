package us.ihmc.octoMap;

import static org.junit.Assert.*;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.node.ColorOcTreeNode;
import us.ihmc.octoMap.ocTree.implementations.ColorOcTree;
import us.ihmc.octoMap.tools.OcTreeNodeTools;

public class ColorTreeTest
{
   public void print_query_info(Point3d query, ColorOcTreeNode node)
   {
      if (node != null)
      {
         System.out.println("occupancy probability at " + query + ":\t " + node.getOccupancy());
         System.out.println("color of node is: " + node.getColor());
      }
      else
         System.out.println("occupancy probability at " + query + ":\t is unknown");
   }

   public static void main(String[] args)
   {
      double res = 0.05; // create empty tree with resolution 0.05 (different from default 0.1 for test)

      ColorOcTree tree = new ColorOcTree(res);
      // insert some measurements of occupied cells
      for (int x = -20; x < 20; x++)
      {
         for (int y = -20; y < 20; y++)
         {
            for (int z = -20; z < 20; z++)
            {
               Point3d endpoint = new Point3d((float) x * 0.05f + 0.01f, (float) y * 0.05f + 0.01f, (float) z * 0.05f + 0.01f);
               ColorOcTreeNode n = tree.updateNode(endpoint, true);
               n.setColor(z * 5 + 100, x * 5 + 100, y * 5 + 100);
            }
         }
      }

      // set inner node colors
      tree.updateInnerOccupancy();
      System.out.println("Tree size: " + tree.size());
      

      // insert some measurements of free cells
      for (int x = -30; x < 30; x++)
      {
         for (int y = -30; y < 30; y++)
         {
            for (int z = -30; z < 30; z++)
            {
               Point3d endpoint = new Point3d((float) x * 0.02f + 2.0f, (float) y * 0.02f + 2.0f, (float) z * 0.02f + 2.0f);
               ColorOcTreeNode n = tree.updateNode(endpoint, false);
               n.setColor(255, 255, 0); // set color to yellow
            }
         }
      }

      // set inner node colors
      tree.updateInnerOccupancy();
      System.out.println("Tree size: " + tree.size());

      // should already be pruned
      assertEquals(tree.size(), tree.calculateNumberOfNodes());

      int initialSize = tree.size();

      assertEquals(1034, initialSize);
      tree.prune();
      assertEquals(tree.size(), tree.calculateNumberOfNodes());
      assertEquals(initialSize, tree.size());

      {
         System.out.println("\nPruning / expansion\n===============================");
         assertEquals(initialSize, tree.size());
         assertEquals(initialSize, tree.calculateNumberOfNodes());
         System.out.println("Initial size: " + tree.size());

         // tree should already be pruned during insertion:
         tree.prune();
         assertEquals(initialSize, tree.size());
         assertEquals(initialSize, tree.calculateNumberOfNodes());

         tree.expand();
         System.out.println("Size after expansion: " + tree.size());
         assertEquals(tree.size(), tree.calculateNumberOfNodes());

         // prune again, should be same as initial size

         tree.prune();
         assertEquals(initialSize, tree.size());
         assertEquals(initialSize, tree.calculateNumberOfNodes());

      }

      // delete / create some nodes
      {
         System.out.println("\nCreating / deleting nodes\n===============================");
         initialSize = tree.size();
         assertEquals(initialSize, tree.calculateNumberOfNodes());
         System.out.println("Initial size: " + initialSize);

         Point3d newCoord = new Point3d(-2.0, -2.0, -2.0);
         ColorOcTreeNode newNode = tree.updateNode(newCoord, true);
         newNode.setColor(255, 0, 0);
         assertTrue(newNode != null);

         int insertedSize = tree.size();
         System.out.println("Size after one insertion: " + insertedSize);
         assertEquals(insertedSize, initialSize + 6);
         assertEquals(insertedSize, tree.calculateNumberOfNodes());

         // find parent of newly inserted node:
         ColorOcTreeNode parentNode = tree.search(newCoord, tree.getTreeDepth() - 1);
         assertTrue(parentNode != null);
         assertTrue(parentNode.hasAtLeastOneChild());

         // only one child exists:
         assertTrue(OcTreeNodeTools.nodeChildExists(parentNode, 0));
         for (int i = 1; i < 8; ++i)
         {
            assertFalse(OcTreeNodeTools.nodeChildExists(parentNode, i));
         }

         tree.deleteNodeChild(parentNode, 0);
         assertEquals(tree.size(), tree.calculateNumberOfNodes());
         assertEquals(tree.size(), insertedSize - 1);

         tree.prune();
         assertEquals(tree.size(), tree.calculateNumberOfNodes());
         assertEquals(tree.size(), insertedSize - 1);

         tree.expandNode(parentNode, 0);
         assertEquals(tree.size(), tree.calculateNumberOfNodes());
         assertEquals(tree.size(), insertedSize + 7);

         assertTrue(tree.pruneNode(parentNode));
         assertEquals(tree.size(), tree.calculateNumberOfNodes());
         assertEquals(tree.size(), insertedSize - 1);
      }
   }

}
