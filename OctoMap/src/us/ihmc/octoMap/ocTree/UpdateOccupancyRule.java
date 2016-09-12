package us.ihmc.octoMap.ocTree;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.EarlyAbortRule;
import us.ihmc.octoMap.ocTree.baseImplementation.OccupancyParametersReadOnly;
import us.ihmc.octoMap.ocTree.baseImplementation.OccupancyTools;
import us.ihmc.octoMap.ocTree.baseImplementation.UpdateRule;

public class UpdateOccupancyRule<NODE extends AbstractOccupancyOcTreeNode<NODE>> implements UpdateRule<NODE>, EarlyAbortRule<NODE>
{
   private float updateLogOdds = Float.NaN;
   private final OccupancyParametersReadOnly parameters;

   public UpdateOccupancyRule(OccupancyParametersReadOnly occupancyParameters)
   {
      this.parameters = occupancyParameters;
   }

   public void setUpdateLogOdds(float updateLogOdds)
   {
      this.updateLogOdds = updateLogOdds;
   }

   @Override
   public void updateLeaf(NODE leafToUpdate, OcTreeKeyReadOnly leafKey)
   {
      float logOdds = OccupancyTools.clipLogOddsToMinMax(parameters, leafToUpdate.getLogOdds() + updateLogOdds);
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
      boolean reachedMaxThreshold = updateLogOdds >= 0.0f && nodeLogOdds >= parameters.getMaxLogOdds();
      boolean reachedMinThreshold = updateLogOdds <= 0.0f && nodeLogOdds <= parameters.getMinLogOdds();
      return reachedMaxThreshold || reachedMinThreshold;
   }
}
