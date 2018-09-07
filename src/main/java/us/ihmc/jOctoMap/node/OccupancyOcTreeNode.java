package us.ihmc.jOctoMap.node;

import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;

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
