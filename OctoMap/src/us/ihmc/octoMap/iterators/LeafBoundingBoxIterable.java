package us.ihmc.octoMap.iterators;

import java.util.Iterator;

import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.iterators.LeafIterable.LeafIterator;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;

/**
 * Bounding-box leaf iterator. This iterator will traverse all leaf nodes
 * within a given bounding box (axis-aligned).
 */
public class LeafBoundingBoxIterable<NODE extends AbstractOcTreeNode<NODE>> implements Iterable<OcTreeSuperNode<NODE>>
{
   private final AbstractOcTreeBase<NODE> tree;
   private int maxDepth;
   private OcTreeBoundingBoxInterface boundingBox;
   private final LeafBoundingBoxIterator<NODE> iterator;

   public LeafBoundingBoxIterable(AbstractOcTreeBase<NODE> tree)
   {
      this(tree, 0, false);
   }

   public LeafBoundingBoxIterable(AbstractOcTreeBase<NODE> tree, int maxDepth)
   {
      this(tree, maxDepth, false);
   }

   public LeafBoundingBoxIterable(AbstractOcTreeBase<NODE> tree, boolean recycleIterator)
   {
      this(tree, 0, recycleIterator);
   }

   public LeafBoundingBoxIterable(AbstractOcTreeBase<NODE> tree, int maxDepth, boolean recycleIterator)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;
      if (recycleIterator)
         iterator = new LeafBoundingBoxIterator<>(tree, maxDepth);
      else
         iterator = null;
   }

   public void setMaxDepth(int maxDepth)
   {
      this.maxDepth = maxDepth;
      if (iterator != null)
         iterator.setMaxDepth(maxDepth);
   }

   public void setBoundingBox(OcTreeBoundingBoxInterface boundingBox)
   {
      this.boundingBox = boundingBox;
      if (iterator != null)
         iterator.setBoundingBox(boundingBox);
   }

   @Override
   public Iterator<OcTreeSuperNode<NODE>> iterator()
   {
      if (iterator == null)
         return new LeafBoundingBoxIterator<>(tree, boundingBox, maxDepth);
      else
      {
         iterator.reset();
         return iterator;
      }
   }

   public static class LeafBoundingBoxIterator<NODE extends AbstractOcTreeNode<NODE>> extends LeafIterator<NODE>
   {
      private OcTreeBoundingBoxInterface boundingBox;

      public LeafBoundingBoxIterator(AbstractOcTreeBase<NODE> tree, int maxDepth)
      {
         this(tree, null, maxDepth);
      }

      public LeafBoundingBoxIterator(AbstractOcTreeBase<NODE> tree, OcTreeBoundingBoxInterface boundingBox, int maxDepth)
      {
         super(tree, maxDepth);
         setBoundingBox(boundingBox);
      }

      public void setBoundingBox(OcTreeBoundingBoxInterface boundingBox)
      {
         this.boundingBox = boundingBox;
      }

      @Override
      public OcTreeSuperNode<NODE> next()
      {
         OcTreeSuperNode<NODE> currentNode = super.next();

         while (!(boundingBox == null || boundingBox.isInBoundingBox(currentNode.getKey())) && hasNext())
            currentNode = super.next();

         return currentNode;
      }
   }
}
