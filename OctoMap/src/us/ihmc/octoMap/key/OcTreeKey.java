package us.ihmc.octoMap.key;

import java.util.Arrays;

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
}
