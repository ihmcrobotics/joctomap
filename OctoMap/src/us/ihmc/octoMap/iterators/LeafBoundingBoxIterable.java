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
   private final int maxDepth;
   private final OcTreeKey minKey = new OcTreeKey();
   private final OcTreeKey maxKey = new OcTreeKey();

   public LeafBoundingBoxIterable(AbstractOcTreeBase<NODE> tree, Point3d min, Point3d max)
   {
      this(tree, min, max, 0);
   }

   public LeafBoundingBoxIterable(AbstractOcTreeBase<NODE> tree, Point3d min, Point3d max, int maxDepth)
   {
      this(tree, tree.coordinateToKey(min), tree.coordinateToKey(max), maxDepth);
   }

   public LeafBoundingBoxIterable(AbstractOcTreeBase<NODE> tree, OcTreeKeyReadOnly min, OcTreeKeyReadOnly max)
   {
      this(tree, min, max, 0);
   }

   public LeafBoundingBoxIterable(AbstractOcTreeBase<NODE> tree, OcTreeKeyReadOnly min, OcTreeKeyReadOnly max, int maxDepth)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;
      minKey.set(min);
      maxKey.set(max);
   }

   @Override
   public Iterator<OcTreeSuperNode<NODE>> iterator()
   {
      return new LeafBoundingBoxIterator<>(tree, minKey, maxKey, maxDepth);
   }

   public static class LeafBoundingBoxIterator<NODE extends AbstractOcTreeNode<NODE>> extends LeafIterator<NODE>
   {
      private final OcTreeKey minKey = new OcTreeKey();
      private final OcTreeKey maxKey = new OcTreeKey();

      public LeafBoundingBoxIterator(AbstractOcTreeBase<NODE> tree, OcTreeKeyReadOnly min, OcTreeKeyReadOnly max, int maxDepth)
      {
         super(tree, maxDepth);
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
