package us.ihmc.jOctoMap.ocTree;

import us.ihmc.jOctoMap.node.OccupancyOcTreeNode;
import us.ihmc.jOctoMap.ocTree.baseImplementation.AbstractOccupancyOcTree;

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
