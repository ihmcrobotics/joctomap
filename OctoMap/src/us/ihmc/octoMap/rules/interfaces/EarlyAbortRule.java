package us.ihmc.octoMap.rules.interfaces;

import us.ihmc.octoMap.node.baseImplementation.AbstractOcTreeNode;

public interface EarlyAbortRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public boolean shouldAbortFullDepthUpdate(NODE nodeToUpdate);
}