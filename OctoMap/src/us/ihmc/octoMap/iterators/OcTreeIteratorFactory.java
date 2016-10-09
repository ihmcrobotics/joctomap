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
      return (node, maxDepth) -> isLeaf(node, maxDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> IteratorSelectionRule<NODE> leavesInsideBoundingBoxOnly(OcTreeBoundingBoxInterface boundingBox)
   {
      return (node, maxDetph) -> isLeaf(node, maxDetph) && (boundingBox == null || boundingBox.isInBoundingBox(node.getKey0(), node.getKey1(), node.getKey2()));
   }
   
   public static <NODE extends AbstractOcTreeNode<NODE>> IteratorSelectionRule<NODE> nodesInsideBoundingBoxOnly(OcTreeBoundingBoxInterface boundingBox)
   {
      return (node, maxDetph) -> boundingBox == null || boundingBox.isInBoundingBox(node.getKey0(), node.getKey1(), node.getKey2());
   }

   private static <NODE extends AbstractOcTreeNode<NODE>> boolean isLeaf(NODE node, int maxDepth)
   {
      return node.getDepth() >= maxDepth || !node.hasAtLeastOneChild();
   }
}
