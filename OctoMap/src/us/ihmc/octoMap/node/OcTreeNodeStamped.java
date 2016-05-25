package us.ihmc.octoMap.node;

public class OcTreeNodeStamped extends AbstractOccupancyOcTreeNode<OcTreeNodeStamped>
{
   private long timestamp;

   public OcTreeNodeStamped()
   {
      super();
   }

   @Override
   public void copyData(OcTreeNodeStamped other)
   {
      super.copyData(other);
      timestamp = other.timestamp;
   }

   public void updateTimestamp()
   {
      timestamp = System.currentTimeMillis();
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   // update occupancy and timesteps of inner nodes 
   @Override
   public void updateOccupancyChildren()
   {
      super.updateOccupancyChildren();
      updateTimestamp();
   }

   public long getTimestamp()
   {
      return timestamp;
   }
}
