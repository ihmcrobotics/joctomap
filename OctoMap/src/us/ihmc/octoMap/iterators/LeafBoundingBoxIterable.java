package us.ihmc.octoMap.iterators;

import java.util.Iterator;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.iterators.LeafIterable.LeafIterator;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
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
   private final OcTreeKey minKey = new OcTreeKey();
   private final OcTreeKey maxKey = new OcTreeKey();
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
         iterator = new LeafBoundingBoxIterator<>(tree, minKey, maxKey, maxDepth);
      else
         iterator = null;
   }

   public void setMaxDepth(int maxDepth)
   {
      this.maxDepth = maxDepth;
      if (iterator != null)
         iterator.setMaxDepth(maxDepth);
   }

   public void setBoundingBox(Point3d min, Point3d max)
   {
      tree.coordinateToKey(min, minKey);
      tree.coordinateToKey(max, maxKey);
      if (iterator != null)
         iterator.setBoundingBox(minKey, maxKey);
   }

   public void setBoundingBox(OcTreeKeyReadOnly min, OcTreeKeyReadOnly max)
   {
      minKey.set(min);
      maxKey.set(max);
      if (iterator != null)
         iterator.setBoundingBox(minKey, maxKey);
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

   public static class LeafBoundingBoxIterator<NODE extends AbstractOcTreeNode<NODE>> extends LeafIterator<NODE>
   {
      private final OcTreeKey minKey = new OcTreeKey();
      private final OcTreeKey maxKey = new OcTreeKey();

      public LeafBoundingBoxIterator(AbstractOcTreeBase<NODE> tree, OcTreeKeyReadOnly min, OcTreeKeyReadOnly max, int maxDepth)
      {
         super(tree, maxDepth);
         setBoundingBox(min, max);
      }

      public void setBoundingBox(OcTreeKeyReadOnly min, OcTreeKeyReadOnly max)
      {
         minKey.set(min);
         maxKey.set(max);
      }

      @Override
      public OcTreeSuperNode<NODE> next()
      {
         OcTreeSuperNode<NODE> currentNode = super.next();

         while (!currentNode.isInsideBoundingBox(minKey, maxKey) && hasNext())
            currentNode = super.next();

         return currentNode;
      }
   }
}
