package us.ihmc.octoMap.iterators;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.octoMap.tools.OcTreeKeyTools;

public class OcTreeSuperNode<NODE extends AbstractOcTreeNode<NODE>>
{
   private final AbstractOcTreeBase<NODE> tree;
   private final NODE node;
   private final OcTreeKey key;
   private final int depth;
   private final int maxDepth;

   private OcTreeSuperNode(AbstractOcTreeBase<NODE> tree, NODE node, OcTreeKey key, int depth, int maxDepth)
   {
      this.tree = tree;
      this.node = node;
      this.key = key;
      this.depth = depth;
      this.maxDepth = maxDepth;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeSuperNode<NODE> createRootSuperNode(AbstractOcTreeBase<NODE> tree, int maxDepth)
   {
      int rootDepth = 0;
      OcTreeKey rootKey = OcTreeKeyTools.getRootKey(tree.getTreeDepth());
      NODE rootNode = tree.getRoot();
      return new OcTreeSuperNode<NODE>(tree, rootNode, rootKey, rootDepth, maxDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeSuperNode<NODE> createChildSuperNode(OcTreeSuperNode<NODE> parentNode, int childIndex)
   {
      AbstractOcTreeBase<NODE> tree = parentNode.tree;
      int maxDepth = parentNode.maxDepth;
      int childDepth = parentNode.depth + 1;
      OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, parentNode.key, childDepth, tree.getTreeDepth());
      NODE childNode = parentNode.node.getChild(childIndex);
      return new OcTreeSuperNode<>(tree, childNode, childKey, childDepth, maxDepth);
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