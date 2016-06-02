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
      int treeDepth = 16;
      double resolution = 0.05;

      int keyMax = OcTreeKeyTools.computeMaximumKeyValue(treeDepth);

      for (int i = 0; i < 1000000; i++)
      {
         int expectedKey = RandomTools.generateRandomInt(random, 0, keyMax);
         double coordinate = keyToCoordinate(expectedKey, resolution, treeDepth);
         int actualKey = coordinateToKey(coordinate, resolution, treeDepth);
         assertEquals(expectedKey, actualKey);
      }

      for (int i = 0; i < 10000; i++)
      {
         OcTreeKey expectedKey = new OcTreeKey(random, keyMax);
         Point3d coordinate = keyToCoordinate(expectedKey, resolution, treeDepth);
         OcTreeKey actualKey = coordinateToKey(coordinate, resolution, treeDepth);
         assertEquals(expectedKey, actualKey);
      }
   }

   @Test
   public void testConvertKeyCoordBackForthAtRandomDepth() throws Exception
   {
      Random random = new Random(654961L);
      int treeDepth = 16;
      double resolution = 0.15;

      for (int i = 0; i < 10000; i++)
      {
         int depth = random.nextInt(treeDepth);
         int keyMax = OcTreeKeyTools.computeMaximumKeyValue(treeDepth);
         int inputKey = RandomTools.generateRandomInt(random, 0, keyMax);
         int expectedKey = adjustKeyAtDepth(inputKey, depth, treeDepth);
         double expectedCoordinate = keyToCoordinate(inputKey, depth, resolution, treeDepth);
         int actualKey = coordinateToKey(expectedCoordinate, depth, resolution, treeDepth);
         if (DEBUG)
         {
            double actualCoordinate = keyToCoordinate(actualKey, depth, resolution, treeDepth);
            System.out.print("expectedCoordinate = " + expectedCoordinate);
            System.out.println(", actualCoordinate = " + actualCoordinate);
            System.out.print("expectedKey = " + expectedKey);
            System.out.println(", actualKey = " + actualKey);
         }
         assertEquals(expectedKey, actualKey);
      }

      for (int i = 0; i < 10000; i++)
      {
         int depth = random.nextInt(treeDepth);
         int keyMax = OcTreeKeyTools.computeMaximumKeyValue(treeDepth);
         OcTreeKey inputKey = new OcTreeKey(random, keyMax);
         OcTreeKey expectedKey = adjustKeyAtDepth(inputKey, depth, treeDepth);
         Point3d coordinate = keyToCoordinate(inputKey, depth, resolution, treeDepth);
         OcTreeKey actualKey = coordinateToKey(coordinate, depth, resolution, treeDepth);
         assertEquals(expectedKey, actualKey);
      }
   }

   @Test
   public void testComputeNodeSize() throws Exception
   {
      for (int treeDepth = 2; treeDepth <= 20; treeDepth++)
      {
         double resolution = 0.05;
         double expectedNodeSize = resolution;

         for (int depth = treeDepth; depth >= 0; depth--)
         {
            double actualNodeSize = OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
            assertEquals(expectedNodeSize, actualNodeSize, expectedNodeSize * 1.0e-7);
            expectedNodeSize *= 2.0;
         }
      }
   }
}
