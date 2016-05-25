package us.ihmc.octoMap.iterators;

import java.util.Iterator;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.OcTreeKey;
import us.ihmc.octoMap.iterators.LeafIterable.LeafIterator;
import us.ihmc.octoMap.node.OcTreeDataNode;
import us.ihmc.octoMap.ocTree.OcTreeBaseImpl;

/**
 * Bounding-box leaf iterator. This iterator will traverse all leaf nodes
 * within a given bounding box (axis-aligned).
 */
public class LeafBoundingBoxIterable<NODE extends OcTreeDataNode<NODE>> implements Iterable<OcTreeSuperNode<NODE>>
{
   private final OcTreeBaseImpl<NODE> tree;
   private final int maxDepth;
   private final OcTreeKey minKey = new OcTreeKey();
   private final OcTreeKey maxKey = new OcTreeKey();

   public LeafBoundingBoxIterable(OcTreeBaseImpl<NODE> tree, Point3d min, Point3d max)
   {
      this(tree, min, max, 0);
   }

   public LeafBoundingBoxIterable(OcTreeBaseImpl<NODE> tree, Point3d min, Point3d max, int maxDepth)
   {
      this(tree, tree.convertCartesianCoordinateToKey(min), tree.convertCartesianCoordinateToKey(max), maxDepth);
   }

   public LeafBoundingBoxIterable(OcTreeBaseImpl<NODE> tree, OcTreeKey min, OcTreeKey max)
   {
      this(tree, min, max, 0);
   }

   public LeafBoundingBoxIterable(OcTreeBaseImpl<NODE> tree, OcTreeKey min, OcTreeKey max, int maxDepth)
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

   public static class LeafBoundingBoxIterator<NODE extends OcTreeDataNode<NODE>> extends LeafIterator<NODE>
   {
      private final OcTreeKey minKey = new OcTreeKey();
      private final OcTreeKey maxKey = new OcTreeKey();

      public LeafBoundingBoxIterator(OcTreeBaseImpl<NODE> tree, OcTreeKey min, OcTreeKey max, int maxDepth)
      {
         super(tree, maxDepth);
         minKey.set(min);
         maxKey.set(max);
      }

      @Override
      public OcTreeSuperNode<NODE> next()
      {
         OcTreeSuperNode<NODE> currentNode = super.next();

         while (!currentNode.isInsideBoundingBox(minKey, maxKey))
            currentNode = super.next();

         return currentNode;
      }
   }
}
