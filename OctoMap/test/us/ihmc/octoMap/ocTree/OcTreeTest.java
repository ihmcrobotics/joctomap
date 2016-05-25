package us.ihmc.octoMap.ocTree;

import static org.junit.Assert.*;

import java.util.HashSet;
import java.util.Random;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.node.OccupancyOcTreeNode;
import us.ihmc.robotics.random.RandomTools;

public class OcTreeTest
{

   @Test
   public void testUpdateNodePoint3dBoolean()
   {
      Random random = new Random(126451L);

      for (int attempt = 0; attempt < 100; attempt++)
      {
         double resolution = 0.15 * random.nextDouble();
         OcTree ocTree = new OcTree(resolution);
         Point3d coordinate = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);

         OccupancyOcTreeNode node = null;

         int numberOfHitUpdates = random.nextInt(12) + 1;

         for (int i = 0; i < numberOfHitUpdates; i++)
         {
            node = ocTree.updateNode(coordinate, true);
            assertTrue(ocTree.isNodeOccupied(node));
         }

         assertTrue(node != null);
         assertTrue(node == ocTree.search(coordinate));

         float logOdds = node.getLogOdds();

         while (ocTree.isNodeOccupied(node))
         {
            ocTree.updateNode(coordinate, false);
            assertTrue(logOdds > node.getLogOdds()); // Ensure the node logodds is decreasing.
            logOdds = node.getLogOdds();
         }
         assertFalse(ocTree.isNodeOccupied(node));

         logOdds = node.getLogOdds();

         while (!ocTree.isNodeOccupied(node))
         {
            ocTree.updateNode(coordinate, true);
            assertTrue(logOdds < node.getLogOdds()); // Ensure the node logodds is increasing.
            logOdds = node.getLogOdds();
         }
         assertTrue(ocTree.isNodeOccupied(node));

         int numberOfLeafs = 0;
         int numberOfOccupiedLeafs = 0;
         int numberOfDuplicates = 0;
         HashSet<OccupancyOcTreeNode> foundNodes = new HashSet<>();

         for (OcTreeSuperNode<OccupancyOcTreeNode> superNode : ocTree.treeIterable())
         {
            if (foundNodes.contains(superNode.getNode()))
               numberOfDuplicates++;
            foundNodes.add(superNode.getNode());
            if (superNode.isLeaf())
            {
               numberOfLeafs++;
               if (ocTree.isNodeOccupied(superNode.getNode()))
                  numberOfOccupiedLeafs++;
            }
            else
            {
               assertTrue(ocTree.isNodeOccupied(superNode.getNode()));
            }
         }

         assertEquals(numberOfDuplicates, 0);
         assertEquals(numberOfOccupiedLeafs, 1);
         assertEquals(numberOfLeafs, 1);
      }
   }
}
