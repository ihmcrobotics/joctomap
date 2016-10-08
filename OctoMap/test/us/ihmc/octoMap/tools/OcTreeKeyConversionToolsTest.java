package us.ihmc.octoMap.tools;

import static org.junit.Assert.*;
import static us.ihmc.octoMap.tools.OcTreeKeyConversionTools.*;
import static us.ihmc.octoMap.tools.OcTreeKeyTools.*;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.testTools.OctoMapRandomTools;

public class OcTreeKeyConversionToolsTest
{
   private static final boolean DEBUG = false;

   @Test
   public void testConvertKeyCoordBackForthMaxDepth() throws Exception
   {
      Random random = new Random(6574961L);
      int treeDepth = 16;
      double resolution = 0.05;

      int keyMax = OcTreeKeyTools.computeMaximumKey(treeDepth);

      for (int i = 0; i < 1000000; i++)
      {
         int expectedKey = random.nextInt(keyMax + 1);
         double coordinate = keyToCoordinate(expectedKey, resolution, treeDepth);
         int actualKey = coordinateToKey(coordinate, resolution, treeDepth);
         assertEquals(expectedKey, actualKey);
      }

      for (int i = 0; i < 10000; i++)
      {
         OcTreeKey expectedKey = new OcTreeKey(random, treeDepth);
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
         int keyMax = OcTreeKeyTools.computeMaximumKey(treeDepth);
         int inputKey = random.nextInt(keyMax + 1);
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
         OcTreeKey inputKey = new OcTreeKey(random, treeDepth);
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

   @Test
   public void testNavigatingWithKey() throws Exception
   {
      double resolution = 0.02;
      int treeDepth = 16;

      // test navigating at lowest level
      int keyMin = 0;
      int keyMax = OcTreeKeyTools.computeMaximumKey(treeDepth);
      double coordinateMin = OcTreeKeyConversionTools.keyToCoordinate(keyMin, resolution, treeDepth);

      for (int key = keyMin + 1; key <= keyMax; key++)
      {
         double expected = coordinateMin + key * resolution;
         double actual = OcTreeKeyConversionTools.keyToCoordinate(key, resolution, treeDepth);
         assertEquals(expected, actual, 1.0e-7);
      }

      // test navigating at all levels
      for (int depth = 1; depth <= treeDepth; depth++)
      {
         keyMin = OcTreeKeyTools.computeMinimumKeyAtDepth(depth, treeDepth);
         keyMax = OcTreeKeyTools.computeMaximumKeyValueAtDepth(depth, treeDepth);
         int keyIncrement = OcTreeKeyTools.computeKeyIntervalAtDepth(depth, treeDepth);
         coordinateMin = OcTreeKeyConversionTools.keyToCoordinate(keyMin, depth, resolution, treeDepth);

         for (int key = keyMin; key <= keyMax; key += keyIncrement)
         {
            double expected = coordinateMin + (key - keyMin) * resolution;
            double actual = OcTreeKeyConversionTools.keyToCoordinate(key, depth, resolution, treeDepth);
            assertEquals(expected, actual, 1.0e-7);
         }
      }
   }

   @Test
   public void testKeyToCoord() throws Exception
   {
      Random random = new Random(21651L);
      int treeDepth = 16;
      double resolution = 0.02;
      Point3d rootCoordinate = new Point3d();
      OcTreeKey rootKey = OcTreeKeyTools.getRootKey(treeDepth);
      testKeyToCoordRecursive(rootKey, rootCoordinate, 0, resolution, treeDepth, random, 3);
   }

   private void testKeyToCoordRecursive(OcTreeKey parentKey, Point3d parentCoordinate, int parentDepth, double resolution, int treeDepth, Random random, int numberOfChildrenToTest)
   {
      if (parentDepth == treeDepth)
         return;

      // This list is used to pick random child indices only once.
      ArrayList<Integer> childIndices = new ArrayList<>();
      for (int childIndex = 0; childIndex < 8; childIndex++)
         childIndices.add(childIndex);

      while(childIndices.size() > (8 - numberOfChildrenToTest))
      {
         int childIndex = childIndices.remove(random.nextInt(childIndices.size()));
         int childDepth = parentDepth + 1;
         OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, parentKey, childDepth, treeDepth);
         Point3d actualChildCoordinate = OcTreeKeyConversionTools.keyToCoordinate(childKey, childDepth, resolution, treeDepth);
         Point3d expectedChildCoordinate = new Point3d(parentCoordinate);
         expectedChildCoordinate.add(computeCoordinateOffset(childIndex, childDepth, resolution, treeDepth));

         assertTrue(expectedChildCoordinate.epsilonEquals(actualChildCoordinate, 1.0e-7));

         testKeyToCoordRecursive(childKey, actualChildCoordinate, childDepth, resolution, treeDepth, random, numberOfChildrenToTest);
      }
   }

   private Vector3d computeCoordinateOffset(int childIndex, int depth, double resolution, int treeDepth)
   {
      Vector3d offsetVector = new Vector3d();
      double computeNodeSize = OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
      double offset = computeNodeSize / 2.0;
      
      offsetVector.setX(offsetVector.getX() + ((childIndex & 1) != 0 ? offset : -offset));
      offsetVector.setY(offsetVector.getY() + ((childIndex & 2) != 0 ? offset : -offset));
      offsetVector.setZ(offsetVector.getZ() + ((childIndex & 4) != 0 ? offset : -offset));
      return offsetVector;
   }

   @Test
   public void testCoordinateToKeyAssertInsideNode() throws Exception
   {
      Random random = new Random(234234L);
      int treeDepth = 16;
      double resolution = 0.02;

      for (int i = 0; i < 100000; i++)
      {
         Point3d randomPoint = OctoMapRandomTools.generateRandomPoint3d(random, 50.0, 50.0, 50.0);
         OcTreeKey key = OcTreeKeyConversionTools.coordinateToKey(randomPoint, resolution, treeDepth);
         Point3d nodeCoordinate = keyToCoordinate(key, resolution, treeDepth);
         boolean xInside = Math.abs(randomPoint.getX() - nodeCoordinate.getX()) <= 0.5 * resolution;
         boolean yInside = Math.abs(randomPoint.getY() - nodeCoordinate.getY()) <= 0.5 * resolution;
         boolean zInside = Math.abs(randomPoint.getZ() - nodeCoordinate.getZ()) <= 0.5 * resolution;
         assertTrue(xInside);
         assertTrue(yInside);
         assertTrue(zInside);
      }
   }
}
