package us.ihmc.octoMap.tools;

import java.util.Random;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyList;
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
      int keyMin = computeMinimumKeyAtDepth(childDepth, treeDepth);

      // x-axis
      if ((childIndex & 1) != 0)
         childKeyToPack.setKey(0, parentKey.getKey(0) + keyMin);
      else
         childKeyToPack.setKey(0, parentKey.getKey(0) - keyMin - (keyMin != 0 ? 0 : 1));
      // y-axis
      if ((childIndex & 2) != 0)
         childKeyToPack.setKey(1, parentKey.getKey(1) + keyMin);
      else
         childKeyToPack.setKey(1, parentKey.getKey(1) - keyMin - (keyMin != 0 ? 0 : 1));
      // z-axis
      if ((childIndex & 4) != 0)
         childKeyToPack.setKey(2, parentKey.getKey(2) + keyMin);
      else
         childKeyToPack.setKey(2, parentKey.getKey(2) - keyMin - (keyMin != 0 ? 0 : 1));
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
      int temp = (int) ((char) (1 << depth) % Character.MAX_VALUE);

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
         int mask = (int) (char) (computeMaximumKey(treeDepth) << level);
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
      if (depth == 0)
         return computeCenterOffsetKey(treeDepth);

      int diff = treeDepth - depth;

      if (diff == 0)
      {
         return key;
      }
      else
      {
         int centerOffsetKey = computeCenterOffsetKey(treeDepth);
         return (int) (char) (((key - centerOffsetKey) >> diff) << diff) + (1 << (diff - 1)) + centerOffsetKey;
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

      OctoMapTools.checkIfDepthValid(depth, treeDepth);

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
         return (int) (char) 1 << (treeDepth - 1);
   }

   /**
    * Computes the key of the root node for a given tree depth
    * @param treeDepth number of levels of the tree.
    * @return root node key.
    */
   public static OcTreeKey getRootKey(int treeDepth)
   {
      int centerOffsetKey = computeCenterOffsetKey(treeDepth);
      return new OcTreeKey(centerOffsetKey, centerOffsetKey, centerOffsetKey);
   }

   /**
    * Computes the key of the root node for a given tree depth
    * @param treeDepth number of levels of the tree.
    * @param rootKeyToPack (output) root node key.
    */
   public static void getRootKey(int treeDepth, OcTreeKey rootKeyToPack)
   {
      int centerOffsetKey = computeCenterOffsetKey(treeDepth);
      rootKeyToPack.set(centerOffsetKey, centerOffsetKey, centerOffsetKey);
   }

   /**
    * Computes the maximum that a key can have at the lowest level for a given tree depth.
    * @param treeDepth number of levels of the tree.
    * @return the key maximum value at the lowest level.
    */
   public static int computeMaximumKey(int treeDepth)
   {
      return (int) (char) ((1 << treeDepth) - 1);
   }

   /**
    * Computes the maximum that a key can have at a given depth for a given tree depth.
    * @param depth level at which the maximum key is to be computed.
    * @param treeDepth number of levels of the tree.
    * @return the key maximum value at the given depth.
    */
   public static int computeMaximumKeyValueAtDepth(int depth, int treeDepth)
   {
      if (depth == treeDepth)
         return computeMaximumKey(treeDepth);
      int keyMinAtDepth = computeMinimumKeyAtDepth(depth, treeDepth);
      int keyMaxAtLowestLevel = computeMaximumKey(treeDepth);
      return (int) (char) (keyMaxAtLowestLevel - keyMinAtDepth + 1);
   }

   /**
    * Computes the minimum that a key can have at a given depth for a given tree depth.
    * @param depth level at which the minimum key is to be computed.
    * @param treeDepth number of levels of the tree.
    * @return the key minimum value at the given depth.
    */
   public static int computeMinimumKeyAtDepth(int depth, int treeDepth)
   {
      if (depth == treeDepth)
         return 0;
      else
         return (int) (char) 1 << (treeDepth - depth - 1);
   }

   /**
    * Computes the key interval between nodes at a given level of a tree.
    * @param depth level at which the key interval is to be computed.
    * @param treeDepth number of levels of the tree.
    * @return the key interval value at the given depth.
    */
   public static int computeKeyIntervalAtDepth(int depth, int treeDepth)
   {
      return (int) (char) (1 << treeDepth - depth);
   }

   public static int computeNumberOfNodesAtDepth(int depth)
   {
      return 1 << depth;
   }

   public static boolean isInsideBoundingBox(OcTreeKey minKey, OcTreeKey maxKey, OcTreeKey keyToTest, int depth, int treeDepth)
   {
      int minKeyValue = OcTreeKeyTools.computeMinimumKeyAtDepth(depth, treeDepth);
      if (keyToTest.getKey(0) < minKey.getKey(0) - minKeyValue)
         return false;
      if (keyToTest.getKey(0) > maxKey.getKey(0) + minKeyValue)
         return false;
      if (keyToTest.getKey(1) < minKey.getKey(1) - minKeyValue)
         return false;
      if (keyToTest.getKey(1) > maxKey.getKey(1) + minKeyValue)
         return false;
      if (keyToTest.getKey(2) < minKey.getKey(2) - minKeyValue)
         return false;
      if (keyToTest.getKey(2) > maxKey.getKey(2) + minKeyValue)
         return false;
      return true;
   }

   public static int generateRandomKey(Random random, int depth, int treeDepth)
   {
      int numberOfNodes = OcTreeKeyTools.computeNumberOfNodesAtDepth(depth);
      int keyMin = OcTreeKeyTools.computeMinimumKeyAtDepth(depth, treeDepth);
      int keyInterval = OcTreeKeyTools.computeKeyIntervalAtDepth(depth, treeDepth);

      int key = (int) (char) (random.nextInt(numberOfNodes) * keyInterval - keyMin);
      OcTreeKeyTools.checkKeyIsValid(key, depth, treeDepth);
      return key;
   }

   public static void checkKeyIsValid(int keyToCheck, int depth, int treeDepth)
   {
      if (!isKeyValid(keyToCheck, depth, treeDepth))
         throw new RuntimeException("The key is invalid: " + keyToCheck + " (at depth: " + depth + ")");
   }

   public static boolean isKeyValid(int keyToCheck, int depth, int treeDepth)
   {
      return keyToCheck >= computeMinimumKeyAtDepth(depth, treeDepth) && keyToCheck <= computeMaximumKeyValueAtDepth(depth, treeDepth);
   }

   public static OcTreeKeyList computeNeighborKeys(OcTreeKey key, int depth, double resolution, int treeDepth, double searchRadius)
   {
      OcTreeKeyList neighborKeys = new OcTreeKeyList();
      computeNeighborKeys(key, depth, resolution, treeDepth, searchRadius, neighborKeys);
      return neighborKeys;
   }

   public static void computeNeighborKeys(OcTreeKey key, int depth, double resolution, int treeDepth, double searchRadius, OcTreeKeyList neighborKeysToPack)
   {
      neighborKeysToPack.clear();
      OctoMapTools.checkIfDepthValid(depth, treeDepth);

      if (depth == 0)
         depth = treeDepth;

      double nodeSize = OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
      int keyInterval = OcTreeKeyTools.computeKeyIntervalAtDepth(depth, treeDepth);

      // generate appropriate keyAtDepth for queried depth
      OcTreeKey keyAtDepth;
      if (depth != treeDepth)
         keyAtDepth = OcTreeKeyTools.adjustKeyAtDepth(key, depth, treeDepth);
      else
         keyAtDepth = new OcTreeKey(key);

      int deltaKeyZMax = (int) (Math.floor(searchRadius / nodeSize));
      
      for (int deltaKeyZ = -deltaKeyZMax; deltaKeyZ <= deltaKeyZMax; deltaKeyZ += 1)
      {
         double radiusAtZ = Math.sqrt(MathTools.square(searchRadius) - MathTools.square(Math.abs(deltaKeyZ) * nodeSize));
         int deltaKeyXMax = (int) (Math.floor(radiusAtZ / nodeSize));
         int k2 = keyAtDepth.getKey(2) + deltaKeyZ * keyInterval;

         if (!isKeyValid(k2, depth, treeDepth)) // Check if we are still in the octree
            continue;

         for (int deltaKeyX = -deltaKeyXMax; deltaKeyX <= deltaKeyXMax; deltaKeyX += 1)
         {
            double maxDistance = Math.sqrt(MathTools.square(radiusAtZ) - MathTools.square(Math.abs(deltaKeyX) * nodeSize));
            int deltaKeyYMax = (int) (Math.floor(maxDistance / nodeSize));
            int k0 = keyAtDepth.getKey(0) + deltaKeyX * keyInterval;

            if (!isKeyValid(k0, depth, treeDepth)) // Check if we are still in the octree
               continue;

            for (int deltaKeyY = -deltaKeyYMax; deltaKeyY <= deltaKeyYMax; deltaKeyY += 1)
            {
               int k1 = keyAtDepth.getKey(1) + deltaKeyY * keyInterval;
               
               if (!isKeyValid(k1, depth, treeDepth)) // Check if we are still in the octree
                  continue;

               neighborKeysToPack.add(k0, k1, k2);
            }
         }
      }
   }
}
