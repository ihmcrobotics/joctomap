package us.ihmc.jOctoMap.rules;

import java.util.Map;

import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;
import us.ihmc.jOctoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.jOctoMap.rules.interfaces.EarlyAbortRule;
import us.ihmc.jOctoMap.rules.interfaces.UpdateRule;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public class UpdateOccupancyRule<NODE extends AbstractOccupancyOcTreeNode<NODE>> implements UpdateRule<NODE>, EarlyAbortRule<NODE>
{
   private float updateLogOdds = Float.NaN;
   private final OccupancyParametersReadOnly parameters;
   private Map<OcTreeKeyReadOnly, Boolean> changedKeys;

   public UpdateOccupancyRule(OccupancyParametersReadOnly occupancyParameters)
   {
      this.parameters = occupancyParameters;
   }

   public void setUpdateLogOdds(float updateLogOdds)
   {
      this.updateLogOdds = updateLogOdds;
   }

   public void detachChangedKeys()
   {
      changedKeys = null;
   }

   public void attachChangedKeys(Map<OcTreeKeyReadOnly, Boolean> changedKeys)
   {
      this.changedKeys = changedKeys;
   }

   @Override
   public void updateLeaf(NODE leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated)
   {
      if (changedKeys != null)
      {
         boolean occupiedBefore = OccupancyTools.isNodeOccupied(parameters, leafToUpdate);
         OccupancyTools.updateNodeLogOdds(parameters, leafToUpdate, updateLogOdds);

         if (nodeJustCreated)
         { // new node
            changedKeys.put(leafKey, true);
         }
         else if (occupiedBefore != OccupancyTools.isNodeOccupied(parameters, leafToUpdate))
         { // occupancy changed, track it
            Boolean changedKeyValue = changedKeys.get(leafKey);
            if (changedKeyValue == null)
               changedKeys.put(leafKey, false);
            else if (changedKeyValue == false)
               changedKeys.remove(leafKey);
         }
      }
      else
      {
         OccupancyTools.updateNodeLogOdds(parameters, leafToUpdate, updateLogOdds);
      }
   }

   @Override
   public void updateInnerNode(NODE innerNodeToUpdate)
   {
      innerNodeToUpdate.updateOccupancyChildren();
   }

   @Override
   public boolean shouldAbortFullDepthUpdate(NODE nodeToUpdate)
   {
      if (nodeToUpdate == null)
         return false;

      // early abort (no change will happen).
      // may cause an overhead in some configuration, but more often helps
      // no change: node already at threshold
      float nodeLogOdds = nodeToUpdate.getLogOdds();
      boolean reachedMaxThreshold = updateLogOdds >= 0.0f && nodeLogOdds >= parameters.getMaxLogOdds();
      boolean reachedMinThreshold = updateLogOdds <= 0.0f && nodeLogOdds <= parameters.getMinLogOdds();
      return reachedMaxThreshold || reachedMinThreshold;
   }
}
