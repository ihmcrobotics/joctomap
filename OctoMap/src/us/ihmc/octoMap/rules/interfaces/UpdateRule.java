package us.ihmc.octoMap.rules.interfaces;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.baseImplementation.AbstractOcTreeNode;

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

   /**
    * Affect the methods using {@link #updateNodeInternal(OcTreeKeyReadOnly, UpdateRule, EarlyAbortRule)}.
    * Whether update of inner nodes is omitted after the update (default: false).
    * This speeds up the insertion, but you need to manually update the inner nodes.
    */
   public default boolean performLazyUpdate()
   {
      return false;
   }
}