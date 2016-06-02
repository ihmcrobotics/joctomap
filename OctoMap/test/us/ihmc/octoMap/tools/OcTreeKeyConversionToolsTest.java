package us.ihmc.octoMap.tools;

import static org.junit.Assert.*;
import static us.ihmc.octoMap.tools.OcTreeKeyConversionTools.coordinateToKey;
import static us.ihmc.octoMap.tools.OcTreeKeyConversionTools.keyToCoordinate;
import static us.ihmc.octoMap.tools.OcTreeKeyTools.adjustKeyAtDepth;

import java.util.Random;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.robotics.random.RandomTools;

public class OcTreeKeyConversionToolsTest
{
   private static final boolean DEBUG = false;

   @Test
   public void testConvertKeyCoordBackForthMaxDepth() throws Exception
   {
      Random random = new Random(6574961L);
      int maxDepth = 16;
      double resolution = 0.05;

      int keyMax = OcTreeKeyTools.computeMaximumKeyValueAtDepth(maxDepth);

      for (int i = 0; i < 1000000; i++)
      {
         int expectedKey = RandomTools.generateRandomInt(random, 0, keyMax);
         double coordinate = keyToCoordinate(expectedKey, resolution, maxDepth);
         int actualKey = coordinateToKey(coordinate, resolution, maxDepth);
         assertEquals(expectedKey, actualKey);
      }

      for (int i = 0; i < 10000; i++)
      {
         OcTreeKey expectedKey = new OcTreeKey(random, keyMax);
         Point3d coordinate = keyToCoordinate(expectedKey, resolution, maxDepth);
         OcTreeKey actualKey = coordinateToKey(coordinate, resolution, maxDepth);
         assertEquals(expectedKey, actualKey);
      }
   }

   @Test
   public void testConvertKeyCoordBackForthAtRandomDepth() throws Exception
   {
      Random random = new Random(654961L);
      int maxDepth = 16;
      double resolution = 0.15;

      for (int i = 0; i < 10000; i++)
      {
         int depth = random.nextInt(maxDepth);
         int keyMax = OcTreeKeyTools.computeMaximumKeyValueAtDepth(maxDepth);
         int inputKey = RandomTools.generateRandomInt(random, 0, keyMax);
         int expectedKey = adjustKeyAtDepth(inputKey, depth, maxDepth);
         double expectedCoordinate = keyToCoordinate(inputKey, depth, resolution, maxDepth);
         int actualKey = coordinateToKey(expectedCoordinate, depth, resolution, maxDepth);
         if (DEBUG)
         {
            double actualCoordinate = keyToCoordinate(actualKey, depth, resolution, maxDepth);
            System.out.print("expectedCoordinate = " + expectedCoordinate);
            System.out.println(", actualCoordinate = " + actualCoordinate);
            System.out.print("expectedKey = " + expectedKey);
            System.out.println(", actualKey = " + actualKey);
         }
         assertEquals(expectedKey, actualKey);
      }

      for (int i = 0; i < 10000; i++)
      {
         int depth = random.nextInt(maxDepth);
         int keyMax = OcTreeKeyTools.computeMaximumKeyValueAtDepth(maxDepth);
         OcTreeKey inputKey = new OcTreeKey(random, keyMax);
         OcTreeKey expectedKey = adjustKeyAtDepth(inputKey, depth, maxDepth);
         Point3d coordinate = keyToCoordinate(inputKey, depth, resolution, maxDepth);
         OcTreeKey actualKey = coordinateToKey(coordinate, depth, resolution, maxDepth);
         assertEquals(expectedKey, actualKey);
      }
   }

   @Test
   public void testComputeNodeSize() throws Exception
   {
      for (int maxDepth = 2; maxDepth <= 20; maxDepth++)
      {
         double resolution = 0.05;
         double expectedNodeSize = resolution;

         for (int depth = maxDepth; depth >= 0; depth--)
         {
            double actualNodeSize = OcTreeKeyConversionTools.computeNodeSize(depth, resolution, maxDepth);
            assertEquals(expectedNodeSize, actualNodeSize, expectedNodeSize * 1.0e-7);
            expectedNodeSize *= 2.0;
         }
      }
   }
}
