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

      for (int treeDepth = 0; treeDepth <= 16; treeDepth++)
      {
         for (int depth = 0; depth <= treeDepth; depth++)
         {
            int expected = (int) (Math.pow(2, treeDepth - depth) / 2.0);
            int actual = OcTreeKeyTools.computeCenterOffsetKey(depth, treeDepth);
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
      
   }

   @Test
   public void testComputeChildKeyGeneratesNotDuplicates() throws Exception
   {
      Random random = new Random(5616L);
      int treeDepth = 16;
      int maxValue = OcTreeKeyTools.computeMaximumKey(treeDepth);

      for (int i = 0; i < 100000; i++)
      {
         KeySet keySet = new KeySet();
         OcTreeKey key = new OcTreeKey(random, maxValue);
         int depth = random.nextInt(treeDepth) + 1;
         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, key, depth, treeDepth);
            assertFalse(keySet.contains(childKey));
            keySet.add(childKey);
         }
      }
   }
}
