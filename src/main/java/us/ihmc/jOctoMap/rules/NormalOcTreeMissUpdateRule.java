package us.ihmc.jOctoMap.rules;

import java.util.Set;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.jOctoMap.rules.interfaces.EarlyAbortRule;
import us.ihmc.jOctoMap.rules.interfaces.UpdateRule;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public class NormalOcTreeMissUpdateRule implements UpdateRule<NormalOcTreeNode>, EarlyAbortRule<NormalOcTreeNode>
{
   private float updateLogOdds = Float.NaN;
   private final OccupancyParametersReadOnly parameters;
   private Set<OcTreeKey> deletedLeaves = null;

   public NormalOcTreeMissUpdateRule(OccupancyParametersReadOnly occupancyParameters)
   {
      this.parameters = occupancyParameters;
   }

   public void setUpdateLogOdds(float updateLogOdds)
   {
      this.updateLogOdds = updateLogOdds;
   }

   public void setDeletedLeavesToUpdate(Set<OcTreeKey> deletedLeavesToUpdate)
   {
      deletedLeaves = deletedLeavesToUpdate;
   }

   @Override
   public boolean shouldAbortFullDepthUpdate(NormalOcTreeNode nodeToUpdate)
   {
      return nodeToUpdate == null;
   }

   @Override
   public void updateLeaf(NormalOcTreeNode leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated)
   {
      OccupancyTools.updateNodeLogOdds(parameters, leafToUpdate, updateLogOdds);
      if (deletedLeaves != null && deleteUpdatedNode(leafToUpdate))
         deletedLeaves.add(leafToUpdate.getKeyCopy());
   }

   @Override
   public void updateInnerNode(NormalOcTreeNode innerNodeToUpdate)
   {
      innerNodeToUpdate.updateOccupancyChildren();
      innerNodeToUpdate.updateHitLocationChildren();
   }

   @Override
   public boolean enableNodeCreation()
   {
      return false;
   }

   @Override
   public boolean deleteUpdatedNode(NormalOcTreeNode nodeJustUpdated)
   {
      return !nodeJustUpdated.hasAtLeastOneChild() && !OccupancyTools.isNodeOccupied(parameters, nodeJustUpdated);
   }
}
