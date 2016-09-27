package us.ihmc.octoMap.tools;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyList;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.testTools.TestOcTree;
import us.ihmc.octoMap.testTools.TestOcTreeNode;
import us.ihmc.octoMap.tools.OcTreeNearestNeighborTools.NeighborActionRule;

public class OcTreeNearestNeighborToolsTest
{
   @Test
   public void testFindRadiusNeighbors() throws Exception
   {
      double resolution = 0.01;
      int treeDepth = 16;
      TestOcTree ocTree = new TestOcTree(resolution, treeDepth);
      Random random = new Random(345435L);
      int numberOfLeavesToCreate = 10000;

      Point3d randomQuery = new Point3d();
      randomQuery.setX(25.0 - 50.0 * random.nextDouble());
      randomQuery.setY(25.0 - 50.0 * random.nextDouble());
      randomQuery.setZ(25.0 - 50.0 * random.nextDouble());

      double sphereRadius = 1.5;
      ocTree.fillRandomlyWithinSphere(random, numberOfLeavesToCreate, randomQuery, sphereRadius, ocTree);

      for (int i = 0; i < 100; i++)
      {
         List<TestOcTreeNode> foundNeighbors = new ArrayList<>();
         OcTreeKeyList foundNeighborKeys = new OcTreeKeyList();
         NeighborActionRule<TestOcTreeNode> recordNeighborsRule = new NeighborActionRule<TestOcTreeNode>()
         {

            @Override
            public void doActionOnNeighbor(TestOcTreeNode node, OcTreeKeyReadOnly nodeKey)
            {
               foundNeighbors.add(node);
               foundNeighborKeys.add(nodeKey);
            }
         };
         double randomRadius = random.nextDouble() * sphereRadius;

         OcTreeNearestNeighborTools.findRadiusNeighbors(ocTree.getRoot(), randomQuery, randomRadius, recordNeighborsRule, resolution, treeDepth);

         List<TestOcTreeNode> expectedNeighbors = new ArrayList<>();
         OcTreeKeyList expectedNeighborKeys = new OcTreeKeyList();

         for (OcTreeSuperNode<TestOcTreeNode> superNode : ocTree.leafIterable())
         {
            Point3d coordinate = superNode.getCoordinate();
            if (coordinate.distance(randomQuery) < randomRadius)
            {
               expectedNeighborKeys.add(superNode.getKey());
               expectedNeighbors.add(superNode.getNode());
            }
         }

         assertEquals(expectedNeighborKeys.size(), foundNeighborKeys.size());
         assertEquals(expectedNeighbors.size(), foundNeighbors.size());

         while (!foundNeighborKeys.isEmpty() || !expectedNeighborKeys.isEmpty())
         {
            OcTreeKeyReadOnly lastExpectedKey = expectedNeighborKeys.getLast();
            if (!foundNeighborKeys.contains(lastExpectedKey))
               fail("Missing a key");
            foundNeighborKeys.remove(lastExpectedKey);
            expectedNeighborKeys.removeLast();
         }

         if (!foundNeighborKeys.isEmpty())
            fail("Found too many keys");
         if (!expectedNeighborKeys.isEmpty())
            fail("Did not find all the neighbor keys");

         while (!foundNeighbors.isEmpty() || !expectedNeighbors.isEmpty())
         {
            TestOcTreeNode firstExpectedNode = expectedNeighbors.get(0);
            if (!foundNeighbors.contains(firstExpectedNode))
               fail("Missing a node");
            foundNeighbors.remove(firstExpectedNode);
            expectedNeighbors.remove(0);
         }

         if (!foundNeighbors.isEmpty())
            fail("Found too many nodes");
         if (!expectedNeighbors.isEmpty())
            fail("Did not find all the neighbors");
      }
   }

   @Test
   public void testFindNearestNeighbor() throws Exception
   {
      double resolution = 0.01;
      int treeDepth = 16;
      TestOcTree ocTree = new TestOcTree(resolution, treeDepth);
      Random random = new Random(345435L);
      int numberOfLeavesToCreate = 10;

      ocTree.fillRandomly(random, numberOfLeavesToCreate, ocTree);

      Point3d randomQuery = new Point3d();

      for (int i = 0; i < 100000; i++)
      {
         randomQuery.setX(0.5 - random.nextDouble());
         randomQuery.setY(0.5 - random.nextDouble());
         randomQuery.setZ(0.5 - random.nextDouble());
         randomQuery.scale(100.0);

         OcTreeKey foundNearestNeighborKey = new OcTreeKey();

         double result = OcTreeNearestNeighborTools.findNearestNeighbor(ocTree.getRoot(), randomQuery, -1.0, Double.POSITIVE_INFINITY,
               foundNearestNeighborKey, resolution, treeDepth);

         assertFalse(Double.isNaN(result));

         OcTreeKeyReadOnly expectedNearestNeighborKey = null;
         double distanceFromQueryToNearestNeighbor = Double.POSITIVE_INFINITY;

         for (OcTreeSuperNode<TestOcTreeNode> superNode : ocTree.leafIterable())
         {
            Point3d coordinate = superNode.getCoordinate();
            double distance = coordinate.distance(randomQuery);
            if (distance < distanceFromQueryToNearestNeighbor)
            {
               distanceFromQueryToNearestNeighbor = distance;
               expectedNearestNeighborKey = superNode.getKey();
            }
         }

         assertEquals(expectedNearestNeighborKey, foundNearestNeighborKey);
      }
   }
}
