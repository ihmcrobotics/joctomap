package us.ihmc.octoMap.ocTree.implementations;

import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.node.OcTreeNodeStamped;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTree;

public class OcTreeStamped extends AbstractOccupancyOcTree<OcTreeNodeStamped>
{

   public OcTreeStamped(double resolution)
   {
      super(resolution);
   }

   /** @return timestamp of last update */
   public long getLastUpdateTime()
   {
      // this value is updated whenever inner nodes are 
      // updated using updateOccupancyChildren()
      return root.getTimestamp();
   }

   public void degradeOutdatedNodes(long timeThreshold)
   {
      long query_time = System.currentTimeMillis();

      for (OcTreeSuperNode<OcTreeNodeStamped> superNode : this)
      {
         OcTreeNodeStamped node = superNode.getNode();
         if (isNodeOccupied(node) && query_time - node.getTimestamp() > timeThreshold)
         {
            integrateMissNoTime(node);
         }
      }
   }

   @Override
   public void updateNodeLogOdds(OcTreeNodeStamped node, float update)
   {
      super.updateNodeLogOdds(node, update);
      node.updateTimestamp();
   }

   public void integrateMissNoTime(OcTreeNodeStamped node)
   {
      updateNodeLogOdds(node, occupancyParameters.getMissProbabilityLogOdds());
   }

   @Override
   protected Class<OcTreeNodeStamped> getNodeClass()
   {
      return OcTreeNodeStamped.class;
   }
}
