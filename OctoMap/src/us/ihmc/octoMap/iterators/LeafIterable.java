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
   private int maxDepth;
   private final LeafIterator<NODE> iterator;

   public LeafIterable(AbstractOcTreeBase<NODE> tree)
   {
      this(tree, 0, false);
   }

   public LeafIterable(AbstractOcTreeBase<NODE> tree, int maxDepth)
   {
      this(tree, maxDepth, false);
   }

   public LeafIterable(AbstractOcTreeBase<NODE> tree, boolean recycleIterator)
   {
      this(tree, 0, recycleIterator);
   }

   public LeafIterable(AbstractOcTreeBase<NODE> tree, int maxDepth, boolean recycleIterator)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;
      if (recycleIterator)
         iterator = new LeafIterator<>(tree, maxDepth);
      else
         iterator = null;
   }

   public void setMaxDepth(int maxDepth)
   {
      this.maxDepth = maxDepth;
      if (iterator != null)
         iterator.setMaxDepth(maxDepth);
   }

   @Override
   public Iterator<OcTreeSuperNode<NODE>> iterator()
   {
      if (iterator == null)
         return new LeafIterator<>(tree, maxDepth);
      else
      {
         iterator.reset();
         return iterator;
      }
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
