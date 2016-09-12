package us.ihmc.octoMap.ocTree;

import static us.ihmc.octoMap.node.OcTreeNodeTools.getNodeChild;

import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
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

   public CountingOcTreeNode updateNode(OcTreeKeyReadOnly key)
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
            createNodeChildUnsafe(curNode, pos);
         }
         // descent tree
         curNode = OcTreeNodeTools.getNodeChild(curNode, pos);
         curNode.increaseCount(); // modify traversed nodes
      }

      return curNode;
   }

   public void getCentersMinHits(List<Point3d> nodeCenters, int minHits)
   {
      OcTreeKey rootKey = OcTreeKeyTools.getRootKey(treeDepth);
      getCentersMinHitsRecurs(nodeCenters, minHits, treeDepth, root, 0, rootKey);
   }

   protected void getCentersMinHitsRecurs(List<Point3d> nodeCenters, int minHits, int maxDepth, CountingOcTreeNode node, int depth, OcTreeKeyReadOnly parentKey)
   {
      if (depth < maxDepth && node.hasAtLeastOneChild())
      {
         OcTreeKey searchKey = new OcTreeKey();

         for (int i = 0; i < 8; ++i)
         {
            if (OcTreeNodeTools.nodeChildExists(node, i))
            {
               OcTreeKeyTools.computeChildKey(i, parentKey, searchKey, depth + 1, treeDepth);
               getCentersMinHitsRecurs(nodeCenters, minHits, maxDepth, getNodeChild(node, i), depth + 1, searchKey);
            }
         }
      }

      else
      { // max level reached

         if (node.getCount() >= minHits)
         {
            nodeCenters.add(keyToCoordinate(parentKey, depth));
         }
      }
   }

   @Override
   protected Class<CountingOcTreeNode> getNodeClass()
   {
      return CountingOcTreeNode.class;
   }
}
