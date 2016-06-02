package us.ihmc.octoMap.tools;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

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
   public void test()
   {
      fail("Not yet implemented");
   }

}
