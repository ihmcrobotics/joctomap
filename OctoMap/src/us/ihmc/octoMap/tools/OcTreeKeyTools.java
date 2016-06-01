package us.ihmc.octoMap.tools;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.robotics.MathTools;

public class OcTreeKeyTools
{
   public static OcTreeKey computeChildKey(int pos, int centerOffsetKey, OcTreeKey parentKey)
   {
      OcTreeKey childKey = new OcTreeKey();
      computeChildKey(pos, centerOffsetKey, parentKey, childKey);
      return childKey;
   }

   /**
    * Computes the key of a child node while traversing the octree, given
    * child index and current key
    *
    * @param[in] pos index of child node (0..7)
    * @param[in] centerOffsetKey constant offset of octree keys
    * @param[in] parentKey current (parent) key
    * @param[out] childKeyToPack  computed child key
    */
   public static void computeChildKey(int pos, int centerOffsetKey, OcTreeKey parentKey, OcTreeKey childKeyToPack)
   {
      // x-axis
      if ((pos & 1) != 0)
         childKeyToPack.setKey(0, parentKey.getKey(0) + centerOffsetKey);
      else
         childKeyToPack.setKey(0, parentKey.getKey(0) - centerOffsetKey - (centerOffsetKey != 0 ? 0 : 1));
      // y-axis
      if ((pos & 2) != 0)
         childKeyToPack.setKey(1, parentKey.getKey(1) + centerOffsetKey);
      else
         childKeyToPack.setKey(1, parentKey.getKey(1) - centerOffsetKey - (centerOffsetKey != 0 ? 0 : 1));
      // z-axis
      if ((pos & 4) != 0)
         childKeyToPack.setKey(2, parentKey.getKey(2) + centerOffsetKey);
      else
         childKeyToPack.setKey(2, parentKey.getKey(2) - centerOffsetKey - (centerOffsetKey != 0 ? 0 : 1));
   }

   /// generate child index (between 0 and 7) from key at given tree depth
   public static int computeChildIndex(OcTreeKey key, int depth)
   {
      int pos = 0;
      int temp = 1 << depth;

      if ((key.getKey(0) & temp) != 0)
         pos += 1;
   
      if ((key.getKey(1) & temp) != 0)
         pos += 2;
   
      if ((key.getKey(2) & temp) != 0)
         pos += 4;
   
      return pos;
   }

   /**
    * Generates a unique key for all keys on a certain level of the tree
    *
    * @param level from the bottom (= tree_depth - depth of key)
    * @param key input indexing key (at lowest resolution / level)
    * @return key corresponding to the input key at the given level
    */
   public static OcTreeKey computeIndexKey(int depth, OcTreeKey key, int maxDepth)
   {
      int level = maxDepth - depth;

      if (level == 0)
      {
         return new OcTreeKey(key);
      }
      else
      {
         int mask = ((1 << maxDepth) - 1) << level;
         OcTreeKey result = new OcTreeKey(key);
         result.setKey(0, result.getKey(0) & mask);
         result.setKey(1, result.getKey(1) & mask);
         result.setKey(2, result.getKey(2) & mask);
         return result;
      }
   }

   /**
    * Adjusts a single key value from the lowest level to correspond to a higher depth (by
    * shifting the key value)
    *
    * @param key Input key, at the lowest tree level
    * @param depth Target depth level for the new key
    * @return Key for the new depth level
    */
   public static int adjustKeyAtDepth(int key, int depth, int maxDepth)
   {
      int diff = maxDepth - depth;
   
      if (diff == 0)
      {
         return key;
      }
      else
      {
         int maxValue = 1 << (maxDepth - 1);
         return (((key - maxValue) >> diff) << diff) + (1 << (diff - 1)) + maxValue;
      }
   }

   /**
    * Adjusts a 3D key from the lowest level to correspond to a higher depth (by
    * shifting the key values)
    *
    * @param key Input key, at the lowest tree level
    * @param depth Target depth level for the new key
    * @return Key for the new depth level
    */
   public static OcTreeKey adjustKeyAtDepth(OcTreeKey key, int depth, int maxDepth)
   {
      if (depth == maxDepth)
         return key;

      MathTools.checkIfLessOrEqual(depth, maxDepth);

      int k0 = adjustKeyAtDepth(key.k[0], depth, maxDepth);
      int k1 = adjustKeyAtDepth(key.k[1], depth, maxDepth);
      int k2 = adjustKeyAtDepth(key.k[2], depth, maxDepth);
      return new OcTreeKey(k0, k1, k2);
   }

   public static int computeMaximumValueFromDepth(int maxDepth)
   {
      return 1 << (maxDepth - 1);
   }
}
