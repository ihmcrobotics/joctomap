package us.ihmc.octoMap.key;

import java.util.Arrays;
import java.util.Random;

import us.ihmc.octoMap.tools.OcTreeKeyTools;

public class OcTreeKey implements OcTreeKeyReadOnly
{
   private char[] k = new char[3];

   public OcTreeKey()
   {
   }

   public OcTreeKey(int a, int b, int c)
   {
      k[0] = (char) a;
      k[1] = (char) b;
      k[2] = (char) c;
   }

   public OcTreeKey(OcTreeKeyReadOnly other)
   {
      set(other);
   }

   public void set(OcTreeKeyReadOnly other)
   {
      for (int i = 0; i < 3; i++)
         k[i] = (char) other.getKey(i);
   }

   public void set(int k0, int k1, int k2)
   {
      k[0] = (char) k0;
      k[1] = (char) k1;
      k[2] = (char) k2;
   }

   public void set(int key[])
   {
      for (int i = 0; i < 3; i++)
         k[i] = (char) key[i];
   }

   public void setKey(int index, int keyValue)
   {
      k[index] = (char) keyValue;
   }

   public void add(OcTreeKeyReadOnly other)
   {
      for (int i = 0; i < 3; i++)
         k[i] += (char) other.getKey(i);
   }
   
   public void add(OcTreeKeyReadOnly k1, OcTreeKeyReadOnly k2)
   {
      for (int i = 0; i < 3; i++)
         k[i] = (char) (k1.getKey(i) + k2.getKey(i));
   }

   public void addKey(int index, int keyValue)
   {
      k[index] += keyValue;
      k[index] = (char) k[index];
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
      return (int) ((long) (k[0]) + 1447L * (long) (k[1]) + 345637L * (long) (k[2]));
   }

   @Override
   public String toString()
   {
      return "OcTreeKey: " + Arrays.toString(k);
   }

   public OcTreeKey(Random random, int treeDepth)
   {
      this(random, treeDepth, treeDepth);
   }

   public OcTreeKey(Random random, int depth, int treeDepth)
   {
      int numberOfNodes = OcTreeKeyTools.computeNumberOfNodesAtDepth(depth);
      int keyMin = OcTreeKeyTools.computeMinimumKeyAtDepth(depth, treeDepth);
      int keyInterval = OcTreeKeyTools.computeKeyIntervalAtDepth(depth, treeDepth);

      for (int i = 0; i < 3; i++)
      {
         int key = (int) (char) (random.nextInt(numberOfNodes) * keyInterval - keyMin);
         OcTreeKeyTools.checkKeyIsValid(key, depth, treeDepth);
         setKey(i, key);
      }
   }
}
