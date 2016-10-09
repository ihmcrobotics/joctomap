package us.ihmc.octoMap.rules.interfaces;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOcTreeNode;

public interface UpdateRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public void updateLeaf(NODE leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated);

   public default boolean enableNodeCreation()
   {
      return true;
   }

   public default boolean deleteUpdatedNode(NODE nodeJustUpdated)
   {
      return false;
   }

   public void updateInnerNode(NODE innerNodeToUpdate);
}