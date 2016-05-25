package us.ihmc.octoMap.ocTree;

import us.ihmc.octoMap.node.OccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTreeBase;

public class OcTree extends AbstractOccupancyOcTreeBase<OccupancyOcTreeNode>
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
}
