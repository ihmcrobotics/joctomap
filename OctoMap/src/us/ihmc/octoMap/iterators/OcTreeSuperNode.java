package us.ihmc.octoMap.iterators;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.OcTreeBaseImpl;
import us.ihmc.octoMap.OcTreeKey;
import us.ihmc.octoMap.node.OcTreeDataNode;
import us.ihmc.octoMap.tools.OctreeKeyTools;

public class OcTreeSuperNode<NODE extends OcTreeDataNode<?>>
{
   private final OcTreeBaseImpl<?, NODE> tree;
   private final NODE node;
   private final OcTreeKey key = new OcTreeKey();
   private final int depth;
   private final int maxDepth;

   /** Constructor for the root node */
   OcTreeSuperNode(OcTreeBaseImpl<?, NODE> tree, int maxDepth)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;

      node = tree.getRoot();
      key.set(tree.getTreeMaximumValue(), tree.getTreeMaximumValue(), tree.getTreeMaximumValue());
      depth = 0;
   }

   /** Constructor for nodes inside the tree */
   @SuppressWarnings("unchecked")
   OcTreeSuperNode(OcTreeBaseImpl<?, NODE> tree, OcTreeSuperNode<NODE> parentNode, int childIndex, int maxDepth)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;
      depth = parentNode.depth + 1;
      int center_offset_key = tree.getTreeMaximumValue() >> depth;
      OctreeKeyTools.computeChildKey(childIndex, center_offset_key, parentNode.key, key);
      node = (NODE) parentNode.node.getChild(childIndex);
   }

   /** @return the center coordinate of this node */
   public Point3d getCoordinate()
   {
      return tree.keyToCoord(key, depth);
   }

   /** @return single coordinate of this node */
   public double getX()
   {
      return tree.keyToCoord(key.getKey(0), depth);
   }

   /** @return single coordinate of this node */
   public double getY()
   {
      return tree.keyToCoord(key.getKey(1), depth);
   }

   /** @return single coordinate of this node */
   public double getZ()
   {
      return tree.keyToCoord(key.getKey(2), depth);
   }

   /** @return the side of the volume occupied by this node */
   public double getSize()
   {
      return tree.getNodeSize(depth);
   }

   /** return depth of this node */
   public int getDepth()
   {
      return depth;
   }

   /** @return the OcTreeKey of this node */
   public OcTreeKey getKey()
   {
      return key;
   }

   /** @return the NODE contained in this. */
   public NODE getNode()
   {
      return node;
   }

   /** @return the OcTreeKey of this node, for nodes with depth != maxDepth */
   public OcTreeKey getIndexKey()
   {
      return OctreeKeyTools.computeIndexKey(tree.getTreeDepth() - depth, key);
   }

   /** @return whether the current node is a leaf, i.e. has no children or is at max level */
   public boolean isLeaf()
   {
      return !node.hasAtLeastOneChild() || depth == maxDepth;
   }

   public boolean isInsideBoundingBox(OcTreeKey minKey, OcTreeKey maxKey)
   {
      int center_offset_key = tree.getTreeMaximumValue() >> depth;
      if (key.getKey(0) < minKey.getKey(0) - center_offset_key) return false;
      if (key.getKey(0) > maxKey.getKey(0) + center_offset_key) return false;
      if (key.getKey(1) < minKey.getKey(1) - center_offset_key) return false;
      if (key.getKey(1) > maxKey.getKey(1) + center_offset_key) return false;
      if (key.getKey(2) < minKey.getKey(2) - center_offset_key) return false;
      if (key.getKey(2) > maxKey.getKey(2) + center_offset_key) return false;
      return true;
   }

   public boolean epsilonEquals(OcTreeSuperNode<NODE> other)
   {
      if (!node.epsilonEquals(other.node))
         return false;
      if (!key.equals(other.key))
         return false;
      return depth == other.depth;
   }
}