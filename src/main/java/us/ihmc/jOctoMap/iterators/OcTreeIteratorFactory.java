package us.ihmc.jOctoMap.iterators;

import java.util.stream.Stream;

import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;
import us.ihmc.jOctoMap.rules.interfaces.IteratorSelectionRule;

public class OcTreeIteratorFactory
{
   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createLeafBoundingBoxIteratable(NODE root, OcTreeBoundingBoxInterface boundingBox)
   {
      OcTreeIterable<NODE> ocTreeIterable = new OcTreeIterable<>(root);
      ocTreeIterable.setRule(leavesInsideBoundingBoxOnly(boundingBox));
      return ocTreeIterable;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createLeafIterable(NODE root)
   {
      OcTreeIterable<NODE> ocTreeIterable = new OcTreeIterable<>(root);
      ocTreeIterable.setRule(leavesOnly());
      return ocTreeIterable;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createLeafIterable(NODE root, int maxDepth)
   {
      OcTreeIterable<NODE> ocTreeIterable = createLeafIterable(root);
      ocTreeIterable.setMaxDepth(maxDepth);
      return ocTreeIterable;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createIterable(NODE root)
   {
      return new OcTreeIterable<>(root);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> OcTreeIterable<NODE> createIterable(NODE root, int maxDepth)
   {
      OcTreeIterable<NODE> ocTreeIterable = new OcTreeIterable<>(root);
      ocTreeIterable.setMaxDepth(maxDepth);
      return ocTreeIterable;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> IteratorSelectionRule<NODE> leavesOnly()
   {
      return (node, maxDepth) -> isLeaf(node, maxDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> IteratorSelectionRule<NODE> leavesInsideBoundingBoxOnly(OcTreeBoundingBoxInterface boundingBox)
   {
      return (node, iteratorMaxDepth) -> isLeaf(node, iteratorMaxDepth) && isNodeInsideBoundingBox(boundingBox, node);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> IteratorSelectionRule<NODE> nodesInsideBoundingBoxOnly(OcTreeBoundingBoxInterface boundingBox)
   {
      return (node, iteratorMaxDepth) -> isNodeInsideBoundingBox(boundingBox, node);
   }

   private static <NODE extends AbstractOcTreeNode<NODE>> boolean isLeaf(NODE node, int iteratorMaxDepth)
   {
      return node.getDepth() >= iteratorMaxDepth || !node.hasAtLeastOneChild();
   }

   private static <NODE extends AbstractOcTreeNode<NODE>> boolean isNodeInsideBoundingBox(OcTreeBoundingBoxInterface boundingBox, NODE node)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(node.getKey0(), node.getKey1(), node.getKey2());
   }

   @SafeVarargs
   public static <NODE extends AbstractOcTreeNode<NODE>> IteratorSelectionRule<NODE> multipleRule(IteratorSelectionRule<NODE>... iteratorSelectionRules)
   {
      return (node, maxDepth) -> !Stream.of(iteratorSelectionRules).filter(rule -> !rule.test(node, maxDepth)).findFirst().isPresent();
   }
}
