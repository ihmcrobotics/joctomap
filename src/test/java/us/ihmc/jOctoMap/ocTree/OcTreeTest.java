package us.ihmc.jOctoMap.ocTree;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.HashSet;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.node.OccupancyOcTreeNode;
import us.ihmc.jOctoMap.tools.JOctoMapRandomTools;

public class OcTreeTest
{
   @Test
   public void testUpdateNodePoint3DBoolean()
   {
      Random random = new Random(126451L);

      for (int attempt = 0; attempt < 100; attempt++)
      {
         double resolution = 0.15 * random.nextDouble();
         OccupancyOcTree ocTree = new OccupancyOcTree(resolution);
         Point3D coordinate = JOctoMapRandomTools.generateRandomPoint3D(random, 10.0, 10.0, 10.0);

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

         for (OccupancyOcTreeNode currentNode : ocTree)
         {
            if (foundNodes.contains(currentNode))
               numberOfDuplicates++;
            foundNodes.add(currentNode);
            if (!currentNode.hasArrayForChildren())
            {
               numberOfLeafs++;
               if (ocTree.isNodeOccupied(currentNode))
                  numberOfOccupiedLeafs++;
            }
            else
            {
               assertTrue(ocTree.isNodeOccupied(currentNode));
            }
         }

         assertEquals(numberOfDuplicates, 0);
         assertEquals(numberOfOccupiedLeafs, 1);
         assertEquals(numberOfLeafs, 1);
      }
   }
}
