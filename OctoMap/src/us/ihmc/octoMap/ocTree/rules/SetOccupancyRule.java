package us.ihmc.octoMap.ocTree.rules;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.rules.interfaces.UpdateRule;

public class SetOccupancyRule<NODE extends AbstractOccupancyOcTreeNode<NODE>> implements UpdateRule<NODE>
{
   private float newLogOdds = Float.NaN;

   public SetOccupancyRule()
   {
   }

   public void setNewLogOdds(float newLogOdds)
   {
      this.newLogOdds = newLogOdds;
   }

   @Override
   public void updateLeaf(NODE leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated)
   {

      leafToUpdate.setLogOdds(newLogOdds);
   }

   @Override
   public void updateInnerNode(NODE innerNodeToUpdate)
   {
      innerNodeToUpdate.updateOccupancyChildren();
   }
}
