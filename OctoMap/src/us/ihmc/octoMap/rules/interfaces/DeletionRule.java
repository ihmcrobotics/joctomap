package us.ihmc.octoMap.rules.interfaces;

import us.ihmc.octoMap.node.AbstractOcTreeNode;

public interface DeletionRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public void updateInnerNodeAfterChildDeletion(NODE node, int indexOfChildDeleted);
}
