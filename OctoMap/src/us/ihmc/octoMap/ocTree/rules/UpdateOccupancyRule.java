package us.ihmc.octoMap.ocTree.rules;

import us.ihmc.octoMap.key.KeyBoolMap;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.rules.interfaces.EarlyAbortRule;
import us.ihmc.octoMap.ocTree.rules.interfaces.UpdateRule;
import us.ihmc.octoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.octoMap.occupancy.OccupancyTools;

public class UpdateOccupancyRule<NODE extends AbstractOccupancyOcTreeNode<NODE>> implements UpdateRule<NODE>, EarlyAbortRule<NODE>
{
   private float updateLogOdds = Float.NaN;
   private final OccupancyParametersReadOnly parameters;
   private KeyBoolMap changedKeys;

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

   public void attachChangedKeys(KeyBoolMap changedKeys)
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
      // early abort (no change will happen).
      // may cause an overhead in some configuration, but more often helps
      // no change: node already at threshold
      float nodeLogOdds = nodeToUpdate.getLogOdds();
      boolean reachedMaxThreshold = updateLogOdds >= 0.0f && nodeLogOdds >= parameters.getMaxLogOdds();
      boolean reachedMinThreshold = updateLogOdds <= 0.0f && nodeLogOdds <= parameters.getMinLogOdds();
      return reachedMaxThreshold || reachedMinThreshold;
   }
}
