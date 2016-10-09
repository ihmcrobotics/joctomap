package us.ihmc.octoMap.iterators;

import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.octoMap.rules.interfaces.IteratorSelectionRule;

public class OcTreeIteratorFactory
{
   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createLeafIteratable(AbstractOcTreeBase<NODE> tree)
   {
      return createLeafIteratable(tree, tree.getTreeDepth(), false);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createLeafIteratable(AbstractOcTreeBase<NODE> tree, int maxDepth)
   {
      return createLeafIteratable(tree, maxDepth, false);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createLeafIteratable(AbstractOcTreeBase<NODE> tree, int maxDepth, boolean recycleIterator)
   {
      return new OcTreeIterable<>(tree, maxDepth, leavesOnly(), recycleIterator);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createIteratable(AbstractOcTreeBase<NODE> tree)
   {
      return createIteratable(tree, tree.getTreeDepth(), false);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createIteratable(AbstractOcTreeBase<NODE> tree, int maxDepth)
   {
      return createIteratable(tree, maxDepth, false);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createIteratable(AbstractOcTreeBase<NODE> tree, int maxDepth, boolean recycleIterator)
   {
      return new OcTreeIterable<>(tree, maxDepth, null, recycleIterator);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> IteratorSelectionRule<NODE> leavesOnly()
   {
      return OcTreeSuperNode::isLeaf;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> IteratorSelectionRule<NODE> leavesInsideBoundingBoxOnly(OcTreeBoundingBoxInterface boundingBox)
   {
      return superNode -> superNode.isLeaf() && (boundingBox == null || boundingBox.isInBoundingBox(superNode.getKey()));
   }
   
   public static <NODE extends AbstractOcTreeNode<NODE>> IteratorSelectionRule<NODE> nodesInsideBoundingBoxOnly(OcTreeBoundingBoxInterface boundingBox)
   {
      return superNode -> boundingBox == null || boundingBox.isInBoundingBox(superNode.getKey());
   }
}
