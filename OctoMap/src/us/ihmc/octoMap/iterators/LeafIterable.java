package us.ihmc.octoMap.iterators;

import java.util.Iterator;

import us.ihmc.octoMap.iterators.OcTreeIterable.OcTreeIterator;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;

/**
 * Iterator to iterate over all leafs of the tree.
 * Inner nodes are skipped.
 *
 */
public class LeafIterable<NODE extends AbstractOcTreeNode<NODE>> implements Iterable<OcTreeSuperNode<NODE>>
{
   private final AbstractOcTreeBase<NODE> tree;
   private final int maxDepth;

   public LeafIterable(AbstractOcTreeBase<NODE> tree)
   {
      this(tree, 0);
   }

   public LeafIterable(AbstractOcTreeBase<NODE> tree, int maxDepth)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;
   }

   @Override
   public Iterator<OcTreeSuperNode<NODE>> iterator()
   {
      return new LeafIterator<>(tree, maxDepth);
   }

   public static class LeafIterator<NODE extends AbstractOcTreeNode<NODE>> extends OcTreeIterator<NODE>
   {
      public LeafIterator(AbstractOcTreeBase<NODE> tree)
      {
         super(tree, 0);
      }

      public LeafIterator(AbstractOcTreeBase<NODE> tree, int maxDepth)
      {
         super(tree, maxDepth);
      }

      @Override
      public OcTreeSuperNode<NODE> next()
      {
         OcTreeSuperNode<NODE> currentNode = super.next();

         while (!currentNode.isLeaf())
            currentNode = super.next();

         return currentNode;
      }
   }
}
