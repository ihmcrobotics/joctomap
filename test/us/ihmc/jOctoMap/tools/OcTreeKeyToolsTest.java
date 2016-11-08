package us.ihmc.jOctoMap.tools;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeySet;
import us.ihmc.jOctoMap.tools.OcTreeKeyTools;

public class OcTreeKeyToolsTest
{
   @Test
   public void testMaximumKeyValueAtDepth() throws Exception
   {
      for (int treeDepth = 0; treeDepth <= 16; treeDepth++)
      {
         int expected = (int) Math.pow(2, treeDepth) - 1;
         int actual = OcTreeKeyTools.computeMaximumKey(treeDepth);
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testCenterOffsetKey() throws Exception
   {
      for (int treeDepth = 0; treeDepth <= 16; treeDepth++)
      {
         int expected = (int) (Math.pow(2, treeDepth) / 2.0);
         int actual = OcTreeKeyTools.computeCenterOffsetKey(treeDepth);
         assertEquals(expected, actual);
      }
   }
   
   @Test
   public void testMinimumKey() throws Exception
   {   
      for (int treeDepth = 0; treeDepth <= 16; treeDepth++)
      {
         for (int depth = 0; depth <= treeDepth; depth++)
         {
            int expected = (int) (Math.pow(2, treeDepth - depth) / 2.0);
            int actual = OcTreeKeyTools.computeMinimumKeyAtDepth(depth, treeDepth);
            assertEquals(expected, actual);

            // Original computation
            int treeMaximumValue = 1 << (treeDepth-1);
            if (treeMaximumValue < 0) treeMaximumValue = 0;
            expected = treeMaximumValue >> depth;
            assertEquals(expected, actual);
         }
      }
   }

   @Test
   public void testAdjustAndChildKey() throws Exception
   {
      Random random = new Random(21651L);
      int treeDepth = 16;
      OcTreeKey rootKey = OcTreeKeyTools.getRootKey(treeDepth);
      testAdjustAndChildKeyRecursive(0, rootKey, treeDepth, random, 3);
   }

   private void testAdjustAndChildKeyRecursive(int parentDepth, OcTreeKey parentKey, int treeDepth, Random random, int numberOfChildrenToTest)
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
         OcTreeKey expected = parentKey;
         OcTreeKey actual = OcTreeKeyTools.adjustKeyAtDepth(childKey, parentDepth, treeDepth);
         assertEquals(expected, actual);
         testAdjustAndChildKeyRecursive(childDepth, childKey, treeDepth, random, numberOfChildrenToTest);
      }
   }

   @Test
   public void testComputeChildKeyGeneratesNotDuplicates() throws Exception
   {
      Random random = new Random(5616L);
      int treeDepth = 16;

      for (int i = 0; i < 100000; i++)
      {
         OcTreeKeySet keySet = new OcTreeKeySet();
         OcTreeKey key = new OcTreeKey(random, random.nextInt(treeDepth), treeDepth);
         int depth = random.nextInt(treeDepth) + 1;
         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, key, depth, treeDepth);
            assertFalse(keySet.contains(childKey));
            keySet.add(childKey);
         }
      }
   }

   @Test
   public void testComputeChildIndexAgainstComputeChildKey() throws Exception
   {
      Random random = new Random(5616L);
      int treeDepth = 16;

      for (int i = 0; i < 100000; i++)
      {
         OcTreeKey currentKey = OcTreeKeyTools.getRootKey(treeDepth);
         OcTreeKey expectedKey = new OcTreeKey(random, treeDepth);

         for (int depth = 0; depth < treeDepth; depth++)
         {
            int childIndex = OcTreeKeyTools.computeChildIndex(expectedKey, depth, treeDepth);
            currentKey = OcTreeKeyTools.computeChildKey(childIndex, currentKey, depth + 1, treeDepth);
         }
         assertEquals(expectedKey, currentKey);
      }
   }
}
