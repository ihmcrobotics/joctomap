package us.ihmc.octoMap.node;

import java.util.Arrays;

public class CountingOcTreeNode extends OcTreeDataNode<Integer>
{
   private int count;

   public CountingOcTreeNode()
   {
   }

   @Override
   public void copyData(OcTreeDataNode<Integer> other)
   {
      count = ((CountingOcTreeNode) other).count;
   }

   public void increaseCount()
   {
      count++;
   }

   public void setCount(int count)
   {
      this.count = count;
   }

   public int getCount()
   {
      return count;
   }

   @Override
   public void updateOccupancyChildren()
   {
      // Do nothing, should already be up-to-date.
   }

   @Override
   public boolean epsilonEquals(OcTreeDataNode<?> other)
   {
      if (!(getClass().isInstance(other)))
         return false;

      return count == ((CountingOcTreeNode) other).count;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": count = " + count + ", children = " + Arrays.toString(getChildrenSimpleNames());
   }
}
