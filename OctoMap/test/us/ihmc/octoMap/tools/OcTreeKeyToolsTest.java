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
         int actual = OcTreeKeyTools.computeMaximumKeyValueAtDepth(depth);
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testCenterOffsetKey() throws Exception
   {
      for (int depth = 0; depth <= 20; depth++)
      {
         int expected = (int) (Math.pow(2, depth) / 2.0);
         int actual = OcTreeKeyTools.computeCenterOffsetKeyAtDepth(depth);
         assertEquals(expected, actual);
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
      int centerOffsetKey = OcTreeKeyTools.computeCenterOffsetKeyAtDepth(maxDepth);
      int maxValue = OcTreeKeyTools.computeMaximumKeyValueAtDepth(maxDepth);

      for (int i = 0; i < 100000; i++)
      {
         KeySet keySet = new KeySet();
         OcTreeKey key = new OcTreeKey(random, maxValue);
         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, centerOffsetKey, key);
            assertFalse(keySet.contains(childKey));
            keySet.add(childKey);
         }
      }
   }
}
