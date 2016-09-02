package us.ihmc.octoMap.node;

public final class OccupancyOcTreeNode extends AbstractOccupancyOcTreeNode<OccupancyOcTreeNode>
{
   public OccupancyOcTreeNode()
   {
      super();
   }

   public OccupancyOcTreeNode(float initialValue)
   {
      super(initialValue);
   }

   @Override
   public void clear()
   {
      super.resetLogOdds();
   }
}
