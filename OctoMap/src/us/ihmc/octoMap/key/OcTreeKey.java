package us.ihmc.octoMap.key;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class OcTreeKey
{
   public int[] k = new int[3];

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

   public void set(int k0, int k1, int k2)
   {
      k[0] = k0;
      k[1] = k1;
      k[2] = k2;
   }

   public void setKey(int index, int keyValue)
   {
      k[index] = keyValue;
   }

   public int getKey(int index)
   {
      return k[index];
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

   @Override
   public String toString()
   {
      return "OcTreeKey: " + Arrays.toString(k);
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

   public static class OcTreeKeyBoundingBox
   {
      
   }
}
