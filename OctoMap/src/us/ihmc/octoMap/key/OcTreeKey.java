package us.ihmc.octoMap.key;

import java.util.Random;

import us.ihmc.octoMap.tools.OcTreeKeyTools;

public class OcTreeKey implements OcTreeKeyReadOnly
{
   private int[] k = new int[3];

   public OcTreeKey()
   {
   }

   public OcTreeKey(int k0, int k1, int k2)
   {
      set(k0, k1, k2);
   }

   public OcTreeKey(OcTreeKeyReadOnly other)
   {
      set(other);
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

   public void add(int k0, int k1, int k2)
   {
      k[0] += k0;
      k[1] += k1;
      k[2] += k2;
   }

   public void addKey(int index, int keyValue)
   {
      k[index] += keyValue;
   }

   public void set(OcTreeKeyReadOnly other)
   {
      set(other.getKey(0), other.getKey(1), other.getKey(2));
   }

   public void set(int key[])
   {
      set(key[0], key[1], key[2]);
   }

   public void add(OcTreeKeyReadOnly other)
   {
      add(other.getKey(0), other.getKey(1), other.getKey(2));
   }

   public void add(OcTreeKeyReadOnly k1, OcTreeKeyReadOnly k2)
   {
      set(k1);
      add(k2);
   }

   @Override
   public int getKey(int index)
   {
      return k[index];
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj instanceof OcTreeKey)
         return equals((OcTreeKey) obj);
      else
         return false;
   }

   @Override
   public boolean equals(OcTreeKey other)
   {
      return k[0] == other.k[0] && k[1] == other.k[1] && k[2] == other.k[2];
   }

   @Override
   public int hashCode()
   {
      // a simple hashing function 
      // explicit casts to size_t to operate on the complete range
      // constanst will be promoted according to C++ standard
      return (int) (k[0] + 1447L * k[1] + 345637L * k[2]);
   }

   @Override
   public String toString()
   {
      return "OcTreeKey: [" + k[0] + ", " + k[1] + ", " + k[2] + "]";
   }

   public OcTreeKey(Random random, int treeDepth)
   {
      this(random, treeDepth, treeDepth);
   }

   public OcTreeKey(Random random, int depth, int treeDepth)
   {
      for (int i = 0; i < 3; i++)
      {
         setKey(i, OcTreeKeyTools.generateRandomKey(random, depth, treeDepth));
      }
   }
}
