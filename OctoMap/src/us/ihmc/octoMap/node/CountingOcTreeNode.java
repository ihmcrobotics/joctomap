package us.ihmc.octoMap.node;

import java.util.Arrays;

public class CountingOcTreeNode extends AbstractOcTreeNode<CountingOcTreeNode>
{
   private int count;

   public CountingOcTreeNode()
   {
   }

   @Override
   public void copyData(CountingOcTreeNode other)
   {
      count = other.count;
   }

   @Override
   public void clear()
   {
      count = 0;
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
   public boolean epsilonEquals(CountingOcTreeNode other)
   {
      return count == other.count;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": count = " + count + ", children = " + Arrays.toString(getChildrenSimpleNames());
   }
}
