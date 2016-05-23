package us.ihmc.octoMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class OcTreeKey
{
   int[] k = new int[3];

   public OcTreeKey()
   {
   }

   public OcTreeKey(int a, int b, int c)
   {
      k[0] = a;
      k[1] = b;
      k[2] = c;
   }

   public OcTreeKey(OcTreeKey other)
   {
      set(other);
   }

   public void set(OcTreeKey other)
   {
      for (int i = 0; i < 3; i++)
         k[i] = other.k[i];
   }

   /**
    * Computes the key of a child node while traversing the octree, given
    * child index and current key
    *
    * @param[in] pos index of child node (0..7)
    * @param[in] center_offset_key constant offset of octree keys
    * @param[in] parent_key current (parent) key
    * @param[out] child_key  computed child key
    */
   public static void computeChildKey(int pos, int center_offset_key, OcTreeKey parent_key, OcTreeKey child_key)
   {
      // x-axis
      if ((pos & 1) != 0)
         child_key.k[0] = parent_key.k[0] + center_offset_key;
      else
         child_key.k[0] = parent_key.k[0] - center_offset_key - (center_offset_key != 0 ? 0 : 1);
      // y-axis
      if ((pos & 2) != 0)
         child_key.k[1] = parent_key.k[1] + center_offset_key;
      else
         child_key.k[1] = parent_key.k[1] - center_offset_key - (center_offset_key != 0 ? 0 : 1);
      // z-axis
      if ((pos & 4) != 0)
         child_key.k[2] = parent_key.k[2] + center_offset_key;
      else
         child_key.k[2] = parent_key.k[2] - center_offset_key - (center_offset_key != 0 ? 0 : 1);
   }

   /// generate child index (between 0 and 7) from key at given tree depth
   public static int computeChildIdx(OcTreeKey key, int depth)
   {
      int pos = 0;
      if ((key.k[0] & (1 << depth)) != 0)
         pos += 1;

      if ((key.k[1] & (1 << depth)) != 0)
         pos += 2;

      if ((key.k[2] & (1 << depth)) != 0)
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
   public static OcTreeKey computeIndexKey(int level, OcTreeKey key) {
     if (level == 0)
       return key;
     else {
       int mask = 65535 << level;
       OcTreeKey result = key;
       result.k[0] &= mask;
       result.k[1] &= mask;
       result.k[2] &= mask;
       return result;
     }
   }

   public boolean equals(OcTreeKey other)
   {
      return Arrays.equals(k, other.k);
   }

   @Override
   public int hashCode()
   {
      // a simple hashing function 
      // explicit casts to size_t to operate on the complete range
      // constanst will be promoted according to C++ standard
      return (int) ((long) (k[0]) + 1447L * (long) (k[1]) + 345637L * (long) (k[2]));
   }

   public static class KeyRay implements Iterable<OcTreeKey>
   {
      private static final int maxSize = 100000;
      private final List<OcTreeKey> ray = new ArrayList<>();

      public KeyRay()
      {
      }

      public KeyRay(KeyRay other)
      {
         ray.clear();
         ray.addAll(other.ray);
      }

      public void addKey(OcTreeKey key)
      {
         ray.add(key);
      }

      public OcTreeKey getFirst()
      {
         return ray.get(0);
      }

      public OcTreeKey getLast()
      {
         return ray.get(ray.size() - 1);
      }

      public void clear()
      {
         ray.clear();
      }

      public int size()
      {
         return ray.size();
      }

      public int sizeMax()
      {
         return maxSize;
      }

      @Override
      public Iterator<OcTreeKey> iterator()
      {
         return ray.iterator();
      }

      public ListIterator<OcTreeKey> reverseIterator()
      {
         return ray.listIterator(ray.size() - 1);
      }
   }

   /**
    * Data structure to efficiently compute the nodes to update from a scan
    * insertion using a hash set.
    * @note you need to use boost::unordered_set instead if your compiler does not
    * yet support tr1!
    */
   public static class KeySet extends HashSet<OcTreeKey>
   {
      private static final long serialVersionUID = 2780317356917541560L;
   }

   /**
    * Data structrure to efficiently track changed nodes as a combination of
    * OcTreeKeys and a bool flag (to denote newly created nodes)
    *
    */
   public static class KeyBoolMap extends HashMap<OcTreeKey, Boolean>
   {
      private static final long serialVersionUID = 2656234567169415329L;
   }
}
