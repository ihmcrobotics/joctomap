package us.ihmc.octoMap.ocTree;

import us.ihmc.octoMap.node.OccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTree;

public class OccupancyOcTree extends AbstractOccupancyOcTree<OccupancyOcTreeNode>
{
   public OccupancyOcTree(double resolution)
   {
      super(resolution);
   }

   @Override
   protected Class<OccupancyOcTreeNode> getNodeClass()
   {
      return OccupancyOcTreeNode.class;
   }
}
