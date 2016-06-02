package us.ihmc.octoMap.tools;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.octoMap.key.KeySet;
import us.ihmc.octoMap.key.OcTreeKey;

public class OcTreeKeyToolsTest
{

   @Test
   public void testMaximumKeyValueAtDepth() throws Exception
   {
      for (int depth = 0; depth <= 20; depth++)
      {
         int expected = (int) Math.pow(2, depth) - 1;
         int actual = OcTreeKeyTools.computeMaximumKeyValue(depth);
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testCenterOffsetKey() throws Exception
   {
      for (int maxDepth = 0; maxDepth <= 20; maxDepth++)
      {
         int expected = (int) (Math.pow(2, maxDepth) / 2.0);
         int actual = OcTreeKeyTools.computeCenterOffsetKey(maxDepth);
         assertEquals(expected, actual);
      }

      for (int maxDepth = 0; maxDepth <= 20; maxDepth++)
      {
         for (int depth = 0; depth <= maxDepth; depth++)
         {
            int expected = (int) (Math.pow(2, maxDepth - depth) / 2.0);
            int actual = OcTreeKeyTools.computeCenterOffsetKey(depth, maxDepth);
            assertEquals(expected, actual);

            // Original computation
            int treeMaximumValue = 1 << (maxDepth-1);
            if (treeMaximumValue < 0) treeMaximumValue = 0;
            expected = treeMaximumValue >> depth;
            assertEquals(expected, actual);
         }
      }
   }

   @Test
   public void testAdjustAndChildKey() throws Exception
   {
      
   }

   @Test
   public void testComputeChildKeyGeneratesNotDuplicates() throws Exception
   {
      Random random = new Random(5616L);
      int maxDepth = 16;
      int maxValue = OcTreeKeyTools.computeMaximumKeyValue(maxDepth);

      for (int i = 0; i < 100000; i++)
      {
         KeySet keySet = new KeySet();
         OcTreeKey key = new OcTreeKey(random, maxValue);
         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            int depth = random.nextInt(maxDepth + 1);
            OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, key, depth, maxDepth);
            assertFalse(keySet.contains(childKey));
            keySet.add(childKey);
         }
      }
   }
}
