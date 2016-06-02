package us.ihmc.octoMap.tools;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.robotics.MathTools;

/**
 * This class provides basic operations on {@linkplain OcTreeKey}.
 * 
 * Here are some notes on keys:
 * <li> Index for a node child is in [0, 7]. Three bits can describe this index (2^0, 2^1, and 2^2).
 * This characteristic is used to figure out where is located the child:
 * 
 *
 */
public class OcTreeKeyTools
{
   /**
    * Computes the 
    * @param childIndex index of child node (0..7)
    * @param centerOffsetKey constant offset of octree keys
    * @param parentKey current (parent) key
    * @return
    */
   public static OcTreeKey computeChildKey(int childIndex, OcTreeKey parentKey, int childDepth, int treeDepth)
   {
      OcTreeKey childKey = new OcTreeKey();
      computeChildKey(childIndex, parentKey, childKey, childDepth, treeDepth);
      return childKey;
   }

   /**
    * Computes the key of a child node while traversing the octree, given
    * child index and current key
    *
    * @param childIndex index of child node (0..7)
    * @param parentKey current (parent) key
    * @param childKeyToPack  computed child key
    */
   public static void computeChildKey(int childIndex, OcTreeKey parentKey, OcTreeKey childKeyToPack, int childDepth, int treeDepth)
   {
      int centerOffsetKey = computeCenterOffsetKey(childDepth, treeDepth);

      // x-axis
      if ((childIndex & 1) != 0)
         childKeyToPack.setKey(0, parentKey.getKey(0) + centerOffsetKey);
      else
         childKeyToPack.setKey(0, parentKey.getKey(0) - centerOffsetKey - (centerOffsetKey != 0 ? 0 : 1));
      // y-axis
      if ((childIndex & 2) != 0)
         childKeyToPack.setKey(1, parentKey.getKey(1) + centerOffsetKey);
      else
         childKeyToPack.setKey(1, parentKey.getKey(1) - centerOffsetKey - (centerOffsetKey != 0 ? 0 : 1));
      // z-axis
      if ((childIndex & 4) != 0)
         childKeyToPack.setKey(2, parentKey.getKey(2) + centerOffsetKey);
      else
         childKeyToPack.setKey(2, parentKey.getKey(2) - centerOffsetKey - (centerOffsetKey != 0 ? 0 : 1));
   }

   /**
    * Generate child index (between 0 and 7) from key at given tree depth
    * @param key
    * @param depth
    * @return
    */
   public static int computeChildIndex(OcTreeKey key, int depth)
   {
      int childIndex = 0;
      int temp = 1 << depth;

      if ((key.getKey(0) & temp) != 0)
         childIndex += 1;

      if ((key.getKey(1) & temp) != 0)
         childIndex += 2;

      if ((key.getKey(2) & temp) != 0)
         childIndex += 4;

      return childIndex;
   }

   /**
    * Generates a unique key for all keys on a certain level of the tree
    * @param key input indexing key (at lowest resolution / level)
    * @param level from the bottom (= tree_depth - depth of key)
    *
    * @return key corresponding to the input key at the given level
    */
   public static OcTreeKey computeIndexKey(OcTreeKey key, int depth, int treeDepth)
   {
      int level = treeDepth - depth;

      if (level == 0)
      {
         return new OcTreeKey(key);
      }
      else
      {
         int mask = computeMaximumKeyValue(treeDepth) << level;
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
   public static int adjustKeyAtDepth(int key, int depth, int treeDepth)
   {
      int diff = treeDepth - depth;

      if (diff == 0)
      {
         return key;
      }
      else
      {
         int centerOffsetKey = computeCenterOffsetKey(treeDepth);
         int returnedKey = (((key - centerOffsetKey) >> diff) << diff) + (1 << (diff - 1)) + centerOffsetKey;
         if (returnedKey == 2 * centerOffsetKey)
            returnedKey = 0;
         return returnedKey;
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
   public static OcTreeKey adjustKeyAtDepth(OcTreeKey key, int depth, int treeDepth)
   {
      if (depth == treeDepth)
         return key;

      MathTools.checkIfLessOrEqual(depth, treeDepth);

      int k0 = adjustKeyAtDepth(key.getKey(0), depth, treeDepth);
      int k1 = adjustKeyAtDepth(key.getKey(1), depth, treeDepth);
      int k2 = adjustKeyAtDepth(key.getKey(2), depth, treeDepth);
      return new OcTreeKey(k0, k1, k2);
   }

   /**
    * Computes the center offset key at depth = 0 and for a given maximum depth (tree depth).
    * It is the key at the center of the tree: all keys are in [0, 2 * centerOffsetKey - 1].
    * It is also the key used for the root node key.
    * @param treeDepth tree depth.
    * @return the center offset key
    */
   public static int computeCenterOffsetKey(int treeDepth)
   {
      if (treeDepth == 0)
         return 0;
      else
         return 1 << (treeDepth - 1);
   }

   public static int computeCenterOffsetKey(int depth, int treeDepth)
   {
      if (depth == treeDepth)
         return 0;
      else
         return 1 << (treeDepth - depth - 1);
   }

   public static OcTreeKey getRootKey(int maxDepth)
   {
      int centerOffsetKey = computeCenterOffsetKey(maxDepth);
      return new OcTreeKey(centerOffsetKey, centerOffsetKey, centerOffsetKey);
   }

   public static void getRootKey(int maxDepth, OcTreeKey rootKeyToPack)
   {
      int centerOffsetKey = computeCenterOffsetKey(maxDepth);
      rootKeyToPack.set(centerOffsetKey, centerOffsetKey, centerOffsetKey);
   }

   /**
    * Computes the maximum that a key can have for a given maximum depth (tree depth).
    * @param treeDepth tree depth.
    * @return the key maximum value
    */
   public static int computeMaximumKeyValue(int treeDepth)
   {
      return (1 << treeDepth) - 1;
   }
}
