package us.ihmc.octoMap.ocTree.baseImplementation;

import us.ihmc.octoMap.node.AbstractOcTreeNode;

public interface EarlyAbortRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public boolean shouldAbortFullDepthUpdate(NODE nodeToUpdate);
}