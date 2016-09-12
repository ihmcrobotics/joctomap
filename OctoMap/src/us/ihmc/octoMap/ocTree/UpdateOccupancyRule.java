package us.ihmc.octoMap.ocTree;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.EarlyAbortRule;
import us.ihmc.octoMap.ocTree.baseImplementation.UpdateRule;

public class UpdateOccupancyRule<NODE extends AbstractOccupancyOcTreeNode<NODE>> implements UpdateRule<NODE>, EarlyAbortRule<NODE>
{
   private float updateLogOdds = Float.NaN;
   private Number maxOccupancyLogOdds;
   private Number minOccupancyLogOdds;

   public UpdateOccupancyRule(Number minOccupancyLogOdds, Number maxOccupancyLogOdds)
   {
      this.minOccupancyLogOdds = minOccupancyLogOdds;
      this.maxOccupancyLogOdds = maxOccupancyLogOdds;
   }

   public void setUpdateLogOdds(float updateLogOdds)
   {
      this.updateLogOdds = updateLogOdds;
   }

   @Override
   public void updateLeaf(NODE leafToUpdate, OcTreeKeyReadOnly leafKey)
   {
      float logOdds = leafToUpdate.getLogOdds() + updateLogOdds;
      if (logOdds < minOccupancyLogOdds.floatValue())
         logOdds = minOccupancyLogOdds.floatValue();
      else if (logOdds > maxOccupancyLogOdds.floatValue())
         logOdds = maxOccupancyLogOdds.floatValue();
      leafToUpdate.setLogOdds(logOdds);
   }

   @Override
   public void updateInnerNode(NODE innerNodeToUpdate)
   {
      innerNodeToUpdate.updateOccupancyChildren();
   }

   @Override
   public boolean shouldAbortFullDepthUpdate(NODE nodeToUpdate)
   {
      // early abort (no change will happen).
      // may cause an overhead in some configuration, but more often helps
      // no change: node already at threshold
      float nodeLogOdds = nodeToUpdate.getLogOdds();
      boolean reachedMaxThreshold = updateLogOdds >= 0.0f && nodeLogOdds >= maxOccupancyLogOdds.floatValue();
      boolean reachedMinThreshold = updateLogOdds <= 0.0f && nodeLogOdds <= minOccupancyLogOdds.floatValue();
      return reachedMaxThreshold || reachedMinThreshold;
   }
}
