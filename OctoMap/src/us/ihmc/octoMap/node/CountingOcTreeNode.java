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
   protected boolean epsilonEqualsInternal(CountingOcTreeNode other, double epsilon)
   {
      return count == other.count;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": count = " + count + ", children = " + Arrays.toString(getChildrenSimpleNames());
   }
}
