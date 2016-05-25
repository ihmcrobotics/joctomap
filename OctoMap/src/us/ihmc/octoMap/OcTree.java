package us.ihmc.octoMap;

import us.ihmc.octoMap.node.OcTreeNode;

public class OcTree extends OccupancyOcTreeBase<OcTreeNode>
{
   public OcTree(double resolution)
   {
      super(resolution);
   }

   @Override
   protected OcTreeNode createRootNode()
   {
      return new OcTreeNode();
   }

   /// virtual constructor: creates a new object of same type
   /// (Covariant return type requires an up-to-date compiler)
   public OcTree create()
   {
      return new OcTree(resolution);
   }

   public String getTreeType()
   {
      return "OcTree";
   }

   
}
