package us.ihmc.octoMap.testTools;

import us.ihmc.octoMap.node.AbstractOcTreeNode;

public class TestOcTreeNode extends AbstractOcTreeNode<TestOcTreeNode>
{
   public TestOcTreeNode()
   {
      super();
   }

   @Override
   public void copyData(TestOcTreeNode other)
   {
   }

   @Override
   public void clear()
   {
   }

   @Override
   protected boolean epsilonEqualsInternal(TestOcTreeNode other, double epsilon)
   {
      return true;
   }
}
