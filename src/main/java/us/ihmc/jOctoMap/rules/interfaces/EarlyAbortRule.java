package us.ihmc.jOctoMap.rules.interfaces;

import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;

public interface EarlyAbortRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public boolean shouldAbortFullDepthUpdate(NODE nodeToUpdate);
}