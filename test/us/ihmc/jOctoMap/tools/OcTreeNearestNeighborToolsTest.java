package us.ihmc.jOctoMap.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyList;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.testTools.TestOcTree;
import us.ihmc.jOctoMap.testTools.TestOcTreeNode;
import us.ihmc.jOctoMap.tools.OcTreeNearestNeighborTools.NeighborActionRule;

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

      Point3D randomQuery = new Point3D();
      randomQuery.setX(25.0 - 50.0 * random.nextDouble());
      randomQuery.setY(25.0 - 50.0 * random.nextDouble());
      randomQuery.setZ(25.0 - 50.0 * random.nextDouble());

      double sphereRadius = 1.5;
      ocTree.fillRandomlyWithinSphere(random, numberOfLeavesToCreate, randomQuery, sphereRadius);

      for (int i = 0; i < 100; i++)
      {
         List<TestOcTreeNode> foundNeighbors = new ArrayList<>();
         OcTreeKeyList foundNeighborKeys = new OcTreeKeyList();
         NeighborActionRule<TestOcTreeNode> recordNeighborsRule = new NeighborActionRule<TestOcTreeNode>()
         {
            @Override
            public void doActionOnNeighbor(TestOcTreeNode node)
            {
               foundNeighbors.add(node);
               foundNeighborKeys.add(node.getKeyCopy());
            }
         };
         double randomRadius = random.nextDouble() * sphereRadius;

         OcTreeNearestNeighborTools.findRadiusNeighbors(ocTree.getRoot(), randomQuery, randomRadius, recordNeighborsRule);

         List<TestOcTreeNode> expectedNeighbors = new ArrayList<>();
         OcTreeKeyList expectedNeighborKeys = new OcTreeKeyList();

         for (TestOcTreeNode node : ocTree)
         {
            Point3D coordinate = new Point3D();
            node.getCoordinate(coordinate);
            if (coordinate.distance(randomQuery) < randomRadius)
            {
               expectedNeighborKeys.add(node.getKeyCopy());
               expectedNeighbors.add(node);
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

      ocTree.fillRandomly(random, numberOfLeavesToCreate);

      Point3D randomQuery = new Point3D();

      for (int i = 0; i < 100000; i++)
      {
         randomQuery.setX(0.5 - random.nextDouble());
         randomQuery.setY(0.5 - random.nextDouble());
         randomQuery.setZ(0.5 - random.nextDouble());
         randomQuery.scale(100.0);

         OcTreeKey foundNearestNeighborKey = new OcTreeKey();

         double result = OcTreeNearestNeighborTools.findNearestNeighbor(ocTree.getRoot(), randomQuery, -1.0, Double.POSITIVE_INFINITY,
               foundNearestNeighborKey);

         assertFalse("iteration: " + i + ". ", Double.isNaN(result));

         OcTreeKeyReadOnly expectedNearestNeighborKey = null;
         double distanceFromQueryToNearestNeighbor = Double.POSITIVE_INFINITY;

         for (TestOcTreeNode node : ocTree)
         {
            Point3D coordinate = new Point3D();
            node.getCoordinate(coordinate);
            double distance = coordinate.distance(randomQuery);
            if (distance < distanceFromQueryToNearestNeighbor)
            {
               distanceFromQueryToNearestNeighbor = distance;
               expectedNearestNeighborKey = node.getKeyCopy();
            }
         }

         assertEquals(expectedNearestNeighborKey, foundNearestNeighborKey);
      }
   }

   @Test
   public void testFindNearestNeighborWithMaximumDistance() throws Exception
   {
      double resolution = 0.01;
      int treeDepth = 16;
      TestOcTree ocTree = new TestOcTree(resolution, treeDepth);
      Random random = new Random(345435L);
      int numberOfLeavesToCreate = 10;

      ocTree.fillRandomly(random, numberOfLeavesToCreate);

      Point3D randomQuery = new Point3D();

      for (int i = 0; i < 100000; i++)
      {
         randomQuery.setX(0.5 - random.nextDouble());
         randomQuery.setY(0.5 - random.nextDouble());
         randomQuery.setZ(0.5 - random.nextDouble());
         randomQuery.scale(100.0);

         double distanceFromQueryToNearestNeighbor = Double.POSITIVE_INFINITY;

         for (TestOcTreeNode node : ocTree)
         {
            Point3D coordinate = new Point3D();
            node.getCoordinate(coordinate);
            double distance = coordinate.distance(randomQuery);
            if (distance < distanceFromQueryToNearestNeighbor)
               distanceFromQueryToNearestNeighbor = distance;
         }

         OcTreeKey foundNearestNeighborKey = new OcTreeKey();
         double result = OcTreeNearestNeighborTools.findNearestNeighbor(ocTree.getRoot(), randomQuery, -1.0, 0.9 * distanceFromQueryToNearestNeighbor, foundNearestNeighborKey);
         assertTrue("iteration: " + i + ". ", Double.isNaN(result));
      }
   }
}
