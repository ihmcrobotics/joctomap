package us.ihmc.octoMap.ocTree;

import static us.ihmc.octoMap.node.OcTreeNodeTools.getNodeChild;

import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.node.CountingOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.octoMap.tools.OcTreeKeyTools;

public class CountingOcTree extends AbstractOcTreeBase<CountingOcTreeNode>
{
   public CountingOcTree(double resolution)
   {
      super(resolution);
   }

   public CountingOcTreeNode updateNode(Point3d coordinate)
   {
      OcTreeKey key = coordinateToKey(coordinate);
      if (key == null)
         return null;
      return updateNode(key);
   }

   public CountingOcTreeNode updateNode(OcTreeKey key)
   {
      CountingOcTreeNode curNode = root;
      curNode.increaseCount();

      // follow or construct nodes down to last level...
      for (int depth = (treeDepth - 1); depth >= 0; depth--)
      {
         int pos = OcTreeKeyTools.computeChildIndex(key, depth);

         // requested node does not exist
         if (!OcTreeNodeTools.nodeChildExists(curNode, pos))
         {
            createNodeChild(curNode, pos);
         }
         // descent tree
         curNode = OcTreeNodeTools.getNodeChild(curNode, pos);
         curNode.increaseCount(); // modify traversed nodes
      }

      return curNode;
   }

   public void getCentersMinHits(List<Point3d> node_centers, int min_hits)
   {
      OcTreeKey root_key = new OcTreeKey(treeMaximumValue, treeMaximumValue, treeMaximumValue);
      getCentersMinHitsRecurs(node_centers, min_hits, treeDepth, root, 0, root_key);
   }

   protected void getCentersMinHitsRecurs(List<Point3d> node_centers, int min_hits, int max_depth, CountingOcTreeNode node, int depth, OcTreeKey parentKey)
   {
      if (depth < max_depth && node.hasAtLeastOneChild())
      {
         int centerOffsetKey = treeMaximumValue >> (depth + 1);
         OcTreeKey searchKey = new OcTreeKey();

         for (int i = 0; i < 8; ++i)
         {
            if (OcTreeNodeTools.nodeChildExists(node, i))
            {
               OcTreeKeyTools.computeChildKey(i, centerOffsetKey, parentKey, searchKey);
               getCentersMinHitsRecurs(node_centers, min_hits, max_depth, getNodeChild(node, i), depth + 1, searchKey);
            }
         }
      }

      else
      { // max level reached

         if (node.getCount() >= min_hits)
         {
            node_centers.add(keyToCoordinate(parentKey, depth));
         }
      }
   }

   @Override
   protected CountingOcTreeNode createRootNode()
   {
      return new CountingOcTreeNode();
   }
}
