package us.ihmc.jOctoMap.ocTree;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Point3d;

import org.apache.commons.lang3.time.StopWatch;
import org.junit.Test;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.jOctoMap.testTools.TestOcTree;
import us.ihmc.jOctoMap.tools.JOctoMapRandomTools;
import us.ihmc.jOctoMap.tools.JOctoMapTools;

public class CloneEpsilonEqualsTest
{
   @Test
   public void testEpsilonEqualsWithTestOcTree() throws Exception
   {
      Random random = new Random(12312L);
      double resolution = 0.02;
      int treeDepth = 16;
      TestOcTree octree1 = new TestOcTree(resolution, treeDepth);
      TestOcTree octree2 = new TestOcTree(resolution, treeDepth);

      // Create 2 identical octrees
      for (int i = 0; i < 1000; i++)
      {
         OcTreeKey key = new OcTreeKey(random, treeDepth);
         octree1.insertNode(key);
         octree2.insertNode(key);
         assertTrue(octree1.epsilonEquals(octree2, 1.0e-7));
      }
      // Make octree2 different from octree1
      octree2.insertNode(new OcTreeKey(random, treeDepth));
      assertFalse(octree1.epsilonEquals(octree2, 1.0e-7));
   }

   @Test
   public void testEpsilonEqualsWithNormalOcTree() throws Exception
   {
      Random random = new Random(12312L);
      double resolution = 0.02;
      NormalOcTree octree1 = new NormalOcTree(resolution);
      NormalOcTree octree2 = new NormalOcTree(resolution);
      octree1.enableParallelComputationForNormals(true);
      octree1.enableParallelInsertionOfMisses(true);
      octree2.enableParallelComputationForNormals(true);
      octree2.enableParallelInsertionOfMisses(true);

      // Create 2 identical octrees
      Point3d sensorOrigin = JOctoMapRandomTools.generateRandomPoint3d(random, 5.0, 5.0, 5.0);
      for (int i = 0; i < 2; i++)
      {
         ScanCollection sweepCollection = JOctoMapRandomTools.generateRandomSweepCollection(random, 50.0f, 50.0f, 50.0f, sensorOrigin, 10, 500);
         octree1.update(sweepCollection);
         octree2.update(sweepCollection);
      }
      assertTrue(octree1.epsilonEquals(octree2, 1.0e-7));

      // Make octree2 different from octree1
      ScanCollection sweepCollection = JOctoMapRandomTools.generateRandomSweepCollection(random, 50.0f, 50.0f, 50.0f, sensorOrigin, 10, 100);
      octree2.update(sweepCollection);
      assertFalse(octree1.epsilonEquals(octree2, 1.0e-7));
   }

   @Test
   public void testCloneWithTestOcTree() throws Exception
   {
      Random random = new Random(12312L);
      double resolution = 0.02;
      int treeDepth = 16;
      TestOcTree octree1 = new TestOcTree(resolution, treeDepth);

      octree1.fillRandomly(random, 5000);
      StopWatch stopWatch = new StopWatch();
      stopWatch.start();
      TestOcTree octree2 = new TestOcTree(octree1);
      System.out.println("Cloning took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()));
      assertEquals(octree1.getNumberOfLeafNodes(), octree2.getNumberOfLeafNodes());
      assertTrue(octree1.epsilonEquals(octree2, 1.0e-7));
   }

   @Test
   public void testCloneWithNormalOcTree() throws Exception
   {
      Random random = new Random(12312L);
      double resolution = 0.02;
      NormalOcTree octree1 = new NormalOcTree(resolution);

      Point3d sensorOrigin = JOctoMapRandomTools.generateRandomPoint3d(random, 5.0, 5.0, 5.0);
      for (int i = 0; i < 2; i++)
      {
         ScanCollection sweepCollection = JOctoMapRandomTools.generateRandomSweepCollection(random, 50.0f, 50.0f, 50.0f, sensorOrigin, 10, 500);
         octree1.update(sweepCollection);
      }

      StopWatch stopWatch = new StopWatch();
      stopWatch.start();
      NormalOcTree octree2 = new NormalOcTree(octree1);
      System.out.println("Cloning took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()) + ", tree size: " + octree1.size());
      assertEquals(octree1.getNumberOfLeafNodes(), octree2.getNumberOfLeafNodes());
      assertTrue(octree1.epsilonEquals(octree2, 1.0e-7));
   }
}
