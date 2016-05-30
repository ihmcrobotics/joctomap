package us.ihmc.octoMap.ocTree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.HashSet;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.key.KeyRay;
import us.ihmc.octoMap.node.OccupancyOcTreeNode;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.time.TimeTools;

public class OcTreeTest
{

   @Test
   public void testInsertPointCloud() throws Exception
   {
      Random random = new Random(126451L);
      double resolution = 0.05;

      Vector3d scanDirection = RandomTools.generateRandomVector(random);
      scanDirection.z = 0.0;
      scanDirection.normalize();

      Point3d origin = new Point3d(0.01, 0.01, 1.02);
      Point3d pointOnSurface = new Point3d(0.01, 0.01, 0.01);
      
      PointCloud pointcloud = new PointCloud();

      double sweepLength = 50.0;
      int nPoints = 20000;

      for (int i = 0; i < nPoints; i++)
      {
         Point3d sweepPoint = new Point3d(pointOnSurface);
         sweepPoint.scaleAdd( (i * sweepLength) / (double) nPoints, scanDirection, pointOnSurface);
         pointcloud.add(sweepPoint);
      }

//      for (int i = 0; i < 10000; i++)
      while(true)
      {

         OcTree octree = new OcTree(resolution);
         long start = System.nanoTime();
         octree.insertPointCloud(pointcloud, origin);
         long endTime = System.nanoTime();
         System.out.println(TimeTools.nanoSecondstoSeconds(endTime - start));
      }
   }

   @Test
   public void testComputeRayKeys() throws Exception
   {
      Random random = new Random(126451L);
      double resolution = 0.002;

      for (int i = 0; i < 10000000; i++)
      {
         Point3d origin = RandomTools.generateRandomPoint(random, 50.0, 50.0, 50.0);
         Point3d end = RandomTools.generateRandomPoint(random, 50.0, 50.0, 50.0);

         OcTree octree = new OcTree(resolution);
         long start = System.nanoTime();
         octree.computeRayKeys(origin, end, new KeyRay());
         long endTime = System.nanoTime();
         System.out.println(TimeTools.nanoSecondstoSeconds(endTime - start));
         
      }
   }

   public static void main(String[] args) throws Exception
   {
      new OcTreeTest().testInsertPointCloud();
   }

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