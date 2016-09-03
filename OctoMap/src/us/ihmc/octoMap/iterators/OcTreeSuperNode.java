package us.ihmc.octoMap.iterators;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.octoMap.tools.OcTreeKeyTools;

public class OcTreeSuperNode<NODE extends AbstractOcTreeNode<NODE>>
{
   private AbstractOcTreeBase<NODE> tree;
   private NODE node;
   private OcTreeKey key = new OcTreeKey();
   private int depth = -1;
   private int maxDepth = -1;

   public OcTreeSuperNode()
   {
      clear();
   }

   public void clear()
   {
      tree = null;
      node = null;
      key.set(0, 0, 0);
      depth = -1;
      maxDepth = -1;
   }

   public void setAsRootSuperNode(AbstractOcTreeBase<NODE> tree, int maxDepth)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;

      node = tree.getRoot();
      depth = 0;
      OcTreeKeyTools.getRootKey(tree.getTreeDepth(), key);
   }

   public void setAsChildSuperNode(OcTreeSuperNode<NODE> parentNode, int childIndex)
   {
      this.tree = parentNode.tree;
      this.maxDepth = parentNode.maxDepth;
      this.depth = parentNode.depth + 1;
      OcTreeKeyTools.computeChildKey(childIndex, parentNode.key, key, depth, tree.getTreeDepth());
      node = parentNode.node.getChild(childIndex);
   }

   /** @return the center coordinate of this node */
   public Point3d getCoordinate()
   {
      return tree.keyToCoordinate(key, depth);
   }

   /** @return single coordinate of this node */
   public double getX()
   {
      return tree.keyToCoordinate(key.getKey(0), depth);
   }

   /** @return single coordinate of this node */
   public double getY()
   {
      return tree.keyToCoordinate(key.getKey(1), depth);
   }

   /** @return single coordinate of this node */
   public double getZ()
   {
      return tree.keyToCoordinate(key.getKey(2), depth);
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
   public OcTreeKeyReadOnly getKey()
   {
      return key;
   }

   /** @return the NODE contained in this. */
   public NODE getNode()
   {
      return node;
   }

   /** @return the OcTreeKey of this node, for nodes with depth != maxDepth */
   public OcTreeKeyReadOnly getIndexKey()
   {
      return OcTreeKeyTools.computeIndexKey(key, depth, tree.getTreeDepth());
   }

   /** @return whether the current node is a leaf, i.e. has no children or is at max level */
   public boolean isLeaf()
   {
      return !node.hasAtLeastOneChild() || depth == maxDepth;
   }

   public boolean isInsideBoundingBox(OcTreeKeyReadOnly minKey, OcTreeKeyReadOnly maxKey)
   {
      return OcTreeKeyTools.isInsideBoundingBox(minKey, maxKey, key, depth, tree.getTreeDepth());
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