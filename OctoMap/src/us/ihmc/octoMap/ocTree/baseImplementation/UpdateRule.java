package us.ihmc.octoMap.ocTree.baseImplementation;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOcTreeNode;

public interface UpdateRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public void updateLeaf(NODE leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated);

   public void updateInnerNode(NODE innerNodeToUpdate);
}