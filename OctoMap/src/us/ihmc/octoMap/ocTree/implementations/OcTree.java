package us.ihmc.octoMap.ocTree.implementations;

import us.ihmc.octoMap.node.OccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTree;

public class OcTree extends AbstractOccupancyOcTree<OccupancyOcTreeNode>
{
   public OcTree(double resolution)
   {
      super(resolution);
   }

   @Override
   protected Class<OccupancyOcTreeNode> getNodeClass()
   {
      return OccupancyOcTreeNode.class;
   }
}
