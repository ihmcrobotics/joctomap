package us.ihmc.jOctoMap.tools;

import static us.ihmc.jOctoMap.tools.OctoMapTools.*;

import java.util.Random;

import us.ihmc.jOctoMap.exceptions.InvalidKeyException;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyList;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;

/**
 * This class provides basic operations on {@linkplain OcTreeKey}.
 * 
 * Here are some notes on keys:
 * <li> Index for a node child is in [0, 7]. Three bits can describe this index (2^0, 2^1, and 2^2).
 * This characteristic is used to figure out where is located the child:
 * 
 *
 */
public abstract class OcTreeKeyTools
{
   /**
    * Computes the 
    * @param childIndex index of child node (0..7)
    * @param centerOffsetKey constant offset of octree keys
    * @param parentKey current (parent) key
    * @return
    */
   public static OcTreeKey computeChildKey(int childIndex, OcTreeKeyReadOnly parentKey, int childDepth, int treeDepth)
   {
      OcTreeKey childKey = new OcTreeKey();
      computeChildKey(childIndex, parentKey, childKey, childDepth, treeDepth);
      return childKey;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeKey computeChildKey(int childIndex, NODE parentNode, int childDepth, int treeDepth)
   {
      OcTreeKey childKey = new OcTreeKey();
      computeChildKey(childIndex, parentNode, childKey, childDepth, treeDepth);
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
   public static void computeChildKey(int childIndex, OcTreeKeyReadOnly parentKey, OcTreeKey childKeyToPack, int childDepth, int treeDepth)
   {
      int k0 = parentKey.getKey(0);
      int k1 = parentKey.getKey(1);
      int k2 = parentKey.getKey(2);
      computeChildKey(childIndex, k0, k1, k2, childKeyToPack, childDepth, treeDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> void computeChildKey(int childIndex, NODE parentNode, OcTreeKey childKeyToPack, int childDepth, int treeDepth)
   {
      int k0 = parentNode.getKey0();
      int k1 = parentNode.getKey1();
      int k2 = parentNode.getKey2();
      computeChildKey(childIndex, k0, k1, k2, childKeyToPack, childDepth, treeDepth);
   }

   private static void computeChildKey(int childIndex, int parentKey0, int parentKey1, int parentKey2, OcTreeKey childKeyToPack, int childDepth, int treeDepth)
   {
      int childKeyMin = computeMinimumKeyAtDepth(childDepth, treeDepth);

      // x-axis
      if ((childIndex & 1) != 0)
         parentKey0 += childKeyMin;
      else
         parentKey0 -= childKeyMin + (childKeyMin != 0 ? 0 : 1);
      // y-axis
      if ((childIndex & 2) != 0)
         parentKey1 += childKeyMin;
      else
         parentKey1 -= childKeyMin + (childKeyMin != 0 ? 0 : 1);
      // z-axis
      if ((childIndex & 4) != 0)
         parentKey2 += childKeyMin;
      else
         parentKey2 -= childKeyMin + (childKeyMin != 0 ? 0 : 1);

      childKeyToPack.setKey(0, adjustToUnsignedNBits(parentKey0, treeDepth));
      childKeyToPack.setKey(1, adjustToUnsignedNBits(parentKey1, treeDepth));
      childKeyToPack.setKey(2, adjustToUnsignedNBits(parentKey2, treeDepth));
   }

   /**
    * Generate child index for node at given depth (between 0 and 7) from key.
    */
   public static int computeChildIndex(OcTreeKeyReadOnly key, int depth, int treeDepth)
   {
      int k0 = key.getKey(0);
      int k1 = key.getKey(1);
      int k2 = key.getKey(2);
      return computeChildIndex(k0, k1, k2, depth, treeDepth);
   }

   /**
    * Generate child index for node at given depth (between 0 and 7) from key.
    */
   public static int computeChildIndex(int k0, int k1, int k2, int depth, int treeDepth)
   {
      OctoMapTools.checkIfDepthValid(depth, treeDepth);

      int childIndex = 0;
      int temp = computeMinimumKeyAtDepth(depth, treeDepth) % computeNumberOfNodesAtDepth(treeDepth);

      if ((k0 & temp) != 0)
         childIndex |= 1;

      if ((k1 & temp) != 0)
         childIndex |= 2;

      if ((k2 & temp) != 0)
         childIndex |= 4;
      return childIndex;
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
         return adjustToUnsignedNBits(adjustToUnsignedNBits(((key - centerOffsetKey) >> diff) << diff, treeDepth) + (1 << (diff - 1)) + centerOffsetKey, treeDepth);
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
   public static OcTreeKey adjustKeyAtDepth(OcTreeKeyReadOnly key, int depth, int treeDepth)
   {
      if (depth == treeDepth)
         return new OcTreeKey(key);

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
         return 1 << (treeDepth - 1);
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
      return (1 << treeDepth) - 1;
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
      return adjustToUnsignedNBits(keyMaxAtLowestLevel - keyMinAtDepth + 1, treeDepth);
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
         return 1 << (treeDepth - depth - 1);
   }

   /**
    * Computes the key interval between nodes at a given level of a tree.
    * @param depth level at which the key interval is to be computed.
    * @param treeDepth number of levels of the tree.
    * @return the key interval value at the given depth.
    */
   public static int computeKeyIntervalAtDepth(int depth, int treeDepth)
   {
      OctoMapTools.checkIfDepthValid(depth, treeDepth);
      return 1 << treeDepth - depth;
   }

   public static int computeNumberOfNodesAtDepth(int depth)
   {
      return 1 << depth;
   }

   public static int generateRandomKey(Random random, int depth, int treeDepth)
   {
      int numberOfNodes = OcTreeKeyTools.computeNumberOfNodesAtDepth(depth);
      int keyMin = OcTreeKeyTools.computeMinimumKeyAtDepth(depth, treeDepth);
      int keyInterval = OcTreeKeyTools.computeKeyIntervalAtDepth(depth, treeDepth);

      int key = adjustToUnsignedNBits(random.nextInt(numberOfNodes) * keyInterval - keyMin, treeDepth);
      OcTreeKeyTools.checkKeyIsValid(key, depth, treeDepth);
      return key;
   }

   public static void checkKeyIsValid(int keyToCheck, int depth, int treeDepth)
   {
      if (!isKeyValid(keyToCheck, depth, treeDepth))
         throw new InvalidKeyException(keyToCheck, depth);
   }

   public static boolean isKeyValid(int keyToCheck, int depth, int treeDepth)
   {
      return keyToCheck >= computeMinimumKeyAtDepth(depth, treeDepth) && keyToCheck <= computeMaximumKeyValueAtDepth(depth, treeDepth);
   }

   public static boolean isKeyValid(OcTreeKey keyToCheck, int depth, int treeDepth)
   {
      int keyMin = computeMinimumKeyAtDepth(depth, treeDepth);
      int keyMax = computeMaximumKeyValueAtDepth(depth, treeDepth);
      for (int i = 0; i < 3; i++)
      {
         int key = keyToCheck.getKey(i);
         if (key < keyMin || key > keyMax)
            return false;
      }
      return true;
   }

   public static OcTreeKeyList computeNeighborKeys(OcTreeKeyReadOnly key, int depth, double resolution, int treeDepth, double searchRadius)
   {
      OcTreeKeyList neighborKeys = new OcTreeKeyList();
      computeNeighborKeys(key, depth, resolution, treeDepth, searchRadius, neighborKeys);
      return neighborKeys;
   }

   public static void computeNeighborKeys(OcTreeKeyReadOnly key, int depth, double resolution, int treeDepth, double searchRadius,
         OcTreeKeyList neighborKeysToPack)
   {
      computeNeighborKeyOffsets(depth, resolution, treeDepth, searchRadius, neighborKeysToPack);

      if (depth == 0)
         depth = treeDepth;

      // generate appropriate keyAtDepth for queried depth
      OcTreeKey keyAtDepth;
      if (depth != treeDepth)
         keyAtDepth = OcTreeKeyTools.adjustKeyAtDepth(key, depth, treeDepth);
      else
         keyAtDepth = new OcTreeKey(key);

      for (int i = 0; i < neighborKeysToPack.size(); i++)
      {
         OcTreeKey currentKey = neighborKeysToPack.get(i);
         currentKey.add(keyAtDepth);
         if (!isKeyValid(currentKey, depth, treeDepth))
            neighborKeysToPack.fastRemove(i);
      }
   }

   public static void computeNeighborKeyOffsets(int depth, double resolution, int treeDepth, double searchRadius, OcTreeKeyList neighborKeyOffsetsToPack)
   {
      neighborKeyOffsetsToPack.clear();
      OctoMapTools.checkIfDepthValid(depth, treeDepth);

      if (depth == 0)
         depth = treeDepth;

      double nodeSize = OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
      int keyInterval = OcTreeKeyTools.computeKeyIntervalAtDepth(depth, treeDepth);

      int deltaKeyZMax = (int) (Math.floor(searchRadius / nodeSize));

      for (int deltaKeyZ = -deltaKeyZMax; deltaKeyZ <= deltaKeyZMax; deltaKeyZ += 1)
      {
         double radiusAtZ = Math.sqrt(square(searchRadius) - square(Math.abs(deltaKeyZ) * nodeSize));
         int deltaKeyXMax = (int) (Math.floor(radiusAtZ / nodeSize));
         int k2 = deltaKeyZ * keyInterval;

         for (int deltaKeyX = -deltaKeyXMax; deltaKeyX <= deltaKeyXMax; deltaKeyX += 1)
         {
            double maxDistance = Math.sqrt(square(radiusAtZ) - square(Math.abs(deltaKeyX) * nodeSize));
            int deltaKeyYMax = (int) (Math.floor(maxDistance / nodeSize));
            int k0 = deltaKeyX * keyInterval;

            for (int deltaKeyY = -deltaKeyYMax; deltaKeyY <= deltaKeyYMax; deltaKeyY += 1)
            {
               int k1 = deltaKeyY * keyInterval;
               if (k0 != 0 || k1 != 0 || k2 != 0)
                  neighborKeyOffsetsToPack.add(k0, k1, k2);
            }
         }
      }
   }

   private static int adjustToUnsignedNBits(int key, int n)
   {
      int maxValue = 1 << n;
      int adjusted = key % maxValue;

      if (adjusted >= 0)
         return adjusted;
      else
         return (adjusted + maxValue) % maxValue;
   }
   
   public static void main(String[] args)
   {
      int blop = 0;
      System.out.println((blop |= 1));
      System.out.println((blop |= 2));
      System.out.println((blop |= 4));
   }
}
