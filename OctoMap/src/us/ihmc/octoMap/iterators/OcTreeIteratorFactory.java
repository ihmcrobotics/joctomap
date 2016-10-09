package us.ihmc.octoMap.iterators;

import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.rules.interfaces.IteratorSelectionRule;

public class OcTreeIteratorFactory
{
   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createLeafIteratable(NODE root)
   {
      OcTreeIterable<NODE> ocTreeIterable = new OcTreeIterable<>(root);
      ocTreeIterable.setRule(leavesOnly());
      return ocTreeIterable;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createLeafIteratable(NODE root, int maxDepth)
   {
      OcTreeIterable<NODE> ocTreeIterable = createLeafIteratable(root);
      ocTreeIterable.setMaxDepth(maxDepth);
      return ocTreeIterable;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createIteratable(NODE root)
   {
      return new OcTreeIterable<>(root);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createIteratable(NODE root, int maxDepth)
   {
      OcTreeIterable<NODE> ocTreeIterable = new OcTreeIterable<NODE>(root);
      ocTreeIterable.setMaxDepth(maxDepth);
      return ocTreeIterable;
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
