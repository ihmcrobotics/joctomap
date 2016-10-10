package us.ihmc.octoMap.node;

import us.ihmc.octoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;

public final class OccupancyOcTreeNode extends AbstractOccupancyOcTreeNode<OccupancyOcTreeNode>
{
   public OccupancyOcTreeNode()
   {
      super();
   }

   @Override
   public void clear()
   {
      super.resetLogOdds();
   }
}
