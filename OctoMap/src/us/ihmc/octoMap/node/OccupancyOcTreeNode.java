package us.ihmc.octoMap.node;

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
