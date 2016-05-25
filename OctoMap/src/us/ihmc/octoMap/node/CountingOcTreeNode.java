package us.ihmc.octoMap.node;

import java.util.Arrays;

public class CountingOcTreeNode extends OcTreeDataNode<Integer>
{
   public CountingOcTreeNode()
   {
      super(0);
   }

   public int getCount()
   {
      return value;
   }

   public void increaseCount()
   {
      value++;
   }

   public void setCount(int count)
   {
      value = count;
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

      return value == (Integer) other.value;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": value = " + value + ", children = " + Arrays.toString(getChildrenSimpleNames());
   }
}
