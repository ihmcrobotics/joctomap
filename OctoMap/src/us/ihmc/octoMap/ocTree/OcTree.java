package us.ihmc.octoMap.ocTree;

import us.ihmc.octoMap.node.OccupancyOcTreeNode;

public class OcTree extends OccupancyOcTreeBase<OccupancyOcTreeNode>
{
   public OcTree(double resolution)
   {
      super(resolution);
   }

   @Override
   protected OccupancyOcTreeNode createRootNode()
   {
      return new OccupancyOcTreeNode();
   }

   /// virtual constructor: creates a new object of same type
   /// (Covariant return type requires an up-to-date compiler)
   public OcTree create()
   {
      return new OcTree(resolution);
   }
}
