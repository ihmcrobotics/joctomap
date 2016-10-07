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
   public boolean epsilonEquals(TestOcTreeNode other)
   {
      return false;
   }
}
