package us.ihmc.octoMap.ocTree.baseImplementation;

import us.ihmc.octoMap.node.AbstractOcTreeNode;

public interface UpdateRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public boolean doLazyEvaluation();

   public void updateLeaf(NODE leafToUpdate);

   public void updateInnerNode(NODE innerNodeToUpdate);
}