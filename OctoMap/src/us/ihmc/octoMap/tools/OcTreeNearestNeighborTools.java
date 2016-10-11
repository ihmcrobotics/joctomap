package us.ihmc.octoMap.tools;

import javax.vecmath.Point3d;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.baseImplementation.AbstractOcTreeNode;

public abstract class OcTreeNearestNeighborTools
{
   public static interface NeighborActionRule<NODE extends AbstractOcTreeNode<NODE>>
   {
      public void doActionOnNeighbor(NODE node);

      default public boolean earlyAbort()
      {
         return false;
      }
   }

   /**
    * Radius neighbor queries where radius determines the maximal radius of reported indices of points in radiusNeighborKeysToPack.
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> void findRadiusNeighbors(NODE rootNode, NODE queryNode, double radius,
         NeighborActionRule<NODE> actionRule)
   {
      double radiusSquared = radius * radius;
      findRadiusNeighbors(rootNode, queryNode.getX(), queryNode.getY(), queryNode.getZ(), radius, radiusSquared, actionRule);
   }

   /**
    * Radius neighbor queries where radius determines the maximal radius of reported indices of points in radiusNeighborKeysToPack.
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> void findRadiusNeighbors(NODE rootNode, Point3d query, double radius,
         NeighborActionRule<NODE> actionRule)
   {
      findRadiusNeighbors(rootNode, query.getX(), query.getY(), query.getZ(), radius, actionRule);
   }

   /**
    * Radius neighbor queries where radius determines the maximal radius of reported indices of points in radiusNeighborKeysToPack.
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> void findRadiusNeighbors(NODE rootNode, double x, double y, double z, double radius,
         NeighborActionRule<NODE> actionRule)
   {
      double radiusSquared = radius * radius;
      findRadiusNeighbors(rootNode, x, y, z, radius, radiusSquared, actionRule);
   }

   private static <NODE extends AbstractOcTreeNode<NODE>> void findRadiusNeighbors(NODE node, double x, double y, double z, double radius, double radiusSquared,
         NeighborActionRule<NODE> actionRule)
   {
      double xNode = node.getX();
      double yNode = node.getY();
      double zNode = node.getZ();

      // if search ball S(q,r) contains octant, simply add point indexes.
      if (contains(node, x, y, z, radiusSquared))
      {
         doActionOnLeavesRecursively(node, actionRule);
         return; // early pruning.
      }

      if (!node.hasAtLeastOneChild())
      {
         double dx = x - xNode;
         double dy = y - yNode;
         double dz = z - zNode;

         double distanceSquared = dx * dx + dy * dy + dz * dz;
         if (distanceSquared < radiusSquared)
            actionRule.doActionOnNeighbor(node);

         return;
      }

      // check whether child nodes are in range.
      for (int childIndex = 0; childIndex < 8; childIndex++)
      {
         NODE child = node.getChild(childIndex);
         if (child == null)
            continue;

         if (!overlaps(child, x, y, z, radius, radiusSquared))
            continue;
         findRadiusNeighbors(child, x, y, z, radius, radiusSquared, actionRule);
         if (actionRule.earlyAbort())
            return;
      }
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> double findNearestNeighbor(NODE rootNode, Point3d query, OcTreeKey nearestNeighborKey,
         double resolution, int treeDepth)
   {
      return findNearestNeighbor(rootNode, query.getX(), query.getY(), query.getZ(), -1.0, Double.POSITIVE_INFINITY, nearestNeighborKey, resolution, treeDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> double findNearestNeighbor(NODE rootNode, Point3d query, double minDistance,
         OcTreeKey nearestNeighborKey, double resolution, int treeDepth)
   {
      return findNearestNeighbor(rootNode, query.getX(), query.getY(), query.getZ(), minDistance, Double.POSITIVE_INFINITY, nearestNeighborKey, resolution,
            treeDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> double findNearestNeighbor(NODE rootNode, Point3d query, double minDistance, double maxDistance,
         OcTreeKey nearestNeighborKey, double resolution, int treeDepth)
   {
      return findNearestNeighbor(rootNode, query.getX(), query.getY(), query.getZ(), minDistance, maxDistance, nearestNeighborKey, resolution, treeDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> double findNearestNeighbor(NODE rootNode, double x, double y, double z, double minDistance,
         double maxDistance, OcTreeKey nearestNeighborKey, double resolution, int treeDepth)
   {
      MutableDouble result = new MutableDouble(maxDistance);

      if (findNearestNeighbor(rootNode, x, y, z, minDistance, result, nearestNeighborKey))
         return result.doubleValue();
      else
         return Double.NaN;
   }

   /** \brief nearest neighbor queries. Using minDistance >= 0, we explicitly disallow self-matches.
    * @return index of nearest neighbor n with Distance::compute(query, n) > minDistance and otherwise -1.
    **/
   private static <NODE extends AbstractOcTreeNode<NODE>> boolean findNearestNeighbor(NODE node, double x, double y, double z, double minDistance,
         MutableDouble maxDistance, OcTreeKey nearestNeighborKey)
   {
      double xNode = node.getX();
      double yNode = node.getY();
      double zNode = node.getZ();

      // 1. first descend to leaf and check in leafs points.
      if (!node.hasAtLeastOneChild())
      {
         double maxDistanceSquared = maxDistance.doubleValue() * maxDistance.doubleValue();
         double minDistanceSquared = (minDistance < 0) ? minDistance : minDistance * minDistance;

         double dx = x - xNode;
         double dy = y - yNode;
         double dz = z - zNode;

         double distanceSquared = dx * dx + dy * dy + dz * dz;
         boolean isBetter = distanceSquared > minDistanceSquared && distanceSquared < maxDistanceSquared;
         if (isBetter)
         {
            node.getKey(nearestNeighborKey);
            maxDistance.setValue(Math.sqrt(distanceSquared));
         }
         return inside(node, x, y, z, maxDistance.doubleValue());
      }

      // determine Morton code for each point...
      int mortonCode = 0;
      if (x > xNode)
         mortonCode |= 1;
      if (y > yNode)
         mortonCode |= 2;
      if (z > zNode)
         mortonCode |= 4;

      NODE child = node.getChild(mortonCode);

      if (child != null)
      {
         if (findNearestNeighbor(child, x, y, z, minDistance, maxDistance, nearestNeighborKey))
            return true;
      }

      // 2. if current best point completely inside, just return.
      double maxDistanceSquared = maxDistance.doubleValue() * maxDistance.doubleValue();

      // 3. check adjacent octants for overlap and check these if necessary.
      for (int childIndex = 0; childIndex < 8; childIndex++)
      {
         if (childIndex == mortonCode)
            continue;

         child = node.getChild(childIndex);

         if (child == null)
            continue;

         if (!overlaps(child, x, y, z, maxDistance.doubleValue(), maxDistanceSquared))
            continue;

         if (findNearestNeighbor(child, x, y, z, minDistance, maxDistance, nearestNeighborKey))
            return true; // early pruning
      }

      // all children have been checked...check if point is inside the current octant...

      return inside(node, x, y, z, maxDistance.doubleValue());
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> void doActionOnLeavesRecursively(NODE node, NeighborActionRule<NODE> actionRule)
   {
      if (!node.hasArrayForChildren())
      {
         actionRule.doActionOnNeighbor(node);
         return;
      }

      for (int childIndex = 0; childIndex < 8; childIndex++)
      {
         NODE childNode = node.getChild(childIndex);
         if (childNode != null)
         {
            doActionOnLeavesRecursively(childNode, actionRule);
            if (actionRule.earlyAbort())
               return;
         }
      }
   }

   /** Test if search ball S(q,r) overlaps with node.
    * <p>
    * From Fast radius neighbor search with an Octree: <a href="https://github.com/jbehley/octree"> GitHub repo</a>, 
    * <a href="http://jbehley.github.io/papers/behley2015icra.pdf"> ICRA Paper </a>.
    * 
    * @param x coordinate of the query point q
    * @param y coordinate of the query point q
    * @param z coordinate of the query point q
    * @param radius  radius r
    * @param squareRadius  "squared" radius r
    * @param key address of the node
    * @param depth current depth in the tree
    * @param resolution resolution of the tree
    * @param treeDepth maximum depth of the tree
    *
    * @return true, if search ball overlaps with node, false otherwise.
    */
   public static boolean overlaps(double x, double y, double z, double radius, double squareRadius, OcTreeKeyReadOnly key, int depth, double resolution,
         int treeDepth)
   {
      double xNode = OcTreeKeyConversionTools.keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      double yNode = OcTreeKeyConversionTools.keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      double zNode = OcTreeKeyConversionTools.keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);

      return overlaps(x, y, z, radius, squareRadius, xNode, yNode, zNode, depth, resolution, treeDepth);
   }

   /** Test if search ball S(q,r) overlaps with node.
    * <p>
    * From Fast radius neighbor search with an Octree: <a href="https://github.com/jbehley/octree"> GitHub repo</a>, 
    * <a href="http://jbehley.github.io/papers/behley2015icra.pdf"> ICRA Paper </a>.
    * 
    * @param x coordinate of the query point q
    * @param y coordinate of the query point q
    * @param z coordinate of the query point q
    * @param radius  radius r
    * @param squareRadius  "squared" radius r
    * @param xNode x coordinate of the node
    * @param yNode y coordinate of the node
    * @param zNode z coordinate of the node
    * @param depth current depth in the tree
    * @param resolution resolution of the tree
    * @param treeDepth maximum depth of the tree
    *
    * @return true, if search ball overlaps with node, false otherwise.
    */
   public static boolean overlaps(double x, double y, double z, double radius, double squareRadius, double xNode, double yNode, double zNode, int depth,
         double resolution, int treeDepth)
   {
      // we exploit the symmetry to reduce the test to testing if its inside the Minkowski sum around the positive quadrant.
      double dx = Math.abs(x - xNode);
      double dy = Math.abs(y - yNode);
      double dz = Math.abs(z - zNode);

      double halfNodeSize = 0.5 * OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);

      // (1) Checking the line region 
      double maxDist = radius + halfNodeSize;

      // a. completely outside, since q' is outside the relevant area.
      if (dx > maxDist || dy > maxDist || dz > maxDist)
         return false;

      // b. inside the line region, one of the coordinates is inside the square.
      if (dx < halfNodeSize || dy < halfNodeSize || dz < halfNodeSize)
         return true;

      // (2) checking the corner region...
      dx -= halfNodeSize;
      dy -= halfNodeSize;
      dz -= halfNodeSize;

      return (dx * dx + dy * dy + dz * dz) < squareRadius;
   }

   /** Test if search ball S(q,r) overlaps with node.
    * <p>
    * From Fast radius neighbor search with an Octree: <a href="https://github.com/jbehley/octree"> GitHub repo</a>, 
    * <a href="http://jbehley.github.io/papers/behley2015icra.pdf"> ICRA Paper </a>.
    * 
    * @param node node that possibly overlaps with the search ball
    * @param x coordinate of the query point q
    * @param y coordinate of the query point q
    * @param z coordinate of the query point q
    * @param radius  radius r
    * @param squareRadius  "squared" radius r
    *
    * @return true, if search ball overlaps with node, false otherwise.
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> boolean overlaps(NODE node, double x, double y, double z, double radius, double squareRadius)
   {
      // we exploit the symmetry to reduce the test to testing if its inside the Minkowski sum around the positive quadrant.
      double dx = Math.abs(x - node.getX());
      double dy = Math.abs(y - node.getY());
      double dz = Math.abs(z - node.getZ());

      double halfNodeSize = 0.5 * node.getSize();

      // (1) Checking the line region 
      double maxDist = radius + halfNodeSize;

      // a. completely outside, since q' is outside the relevant area.
      if (dx > maxDist || dy > maxDist || dz > maxDist)
         return false;

      // b. inside the line region, one of the coordinates is inside the square.
      if (dx < halfNodeSize || dy < halfNodeSize || dz < halfNodeSize)
         return true;

      // (2) checking the corner region...
      dx -= halfNodeSize;
      dy -= halfNodeSize;
      dz -= halfNodeSize;

      return (dx * dx + dy * dy + dz * dz) < squareRadius;
   }

   /** Test if search ball S(q,r) contains node
    * <p>
    * From Fast radius neighbor search with an Octree: <a href="https://github.com/jbehley/octree"> GitHub repo</a>, 
    * <a href="http://jbehley.github.io/papers/behley2015icra.pdf"> ICRA Paper </a>.
    *
    * @param x coordinate of the query point q
    * @param y coordinate of the query point q
    * @param z coordinate of the query point q
    * @param squareRadius "squared" radius r
    * @param xNode x coordinate of the node
    * @param yNode y coordinate of the node
    * @param zNode z coordinate of the node
    * @param depth current depth in the tree
    * @param resolution resolution of the tree
    * @param treeDepth maximum depth of the tree
    *
    * @return true, if search ball overlaps with node, false otherwise.
    */
   public static boolean contains(double x, double y, double z, double squareRadius, OcTreeKeyReadOnly key, int depth, double resolution, int treeDepth)
   {
      // we exploit the symmetry to reduce the test to test whether the farthest corner is inside the search ball.
      double xNode = OcTreeKeyConversionTools.keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      double yNode = OcTreeKeyConversionTools.keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      double zNode = OcTreeKeyConversionTools.keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);
      return contains(x, y, z, squareRadius, xNode, yNode, zNode, depth, resolution, treeDepth);
   }

   /** Test if search ball S(q,r) contains node
    * <p>
    * From Fast radius neighbor search with an Octree: <a href="https://github.com/jbehley/octree"> GitHub repo</a>, 
    * <a href="http://jbehley.github.io/papers/behley2015icra.pdf"> ICRA Paper </a>.
    *
    * @param x coordinate of the query point q
    * @param y coordinate of the query point q
    * @param z coordinate of the query point q
    * @param squareRadius "squared" radius r
    * @param key address of the node
    * @param depth current depth in the tree
    * @param resolution resolution of the tree
    * @param treeDepth maximum depth of the tree
    *
    * @return true, if search ball overlaps with node, false otherwise.
    */
   public static boolean contains(double x, double y, double z, double squareRadius, double xNode, double yNode, double zNode, int depth, double resolution,
         int treeDepth)
   {
      // we exploit the symmetry to reduce the test to test whether the farthest corner is inside the search ball.
      x = Math.abs(x - xNode);
      y = Math.abs(y - yNode);
      z = Math.abs(z - zNode);

      double halfNodeSize = 0.5 * OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);

      // reminder: (x, y, z) - (-halfNodeSize, -halfNodeSize, -halfNodeSize) = (x, y, z) + (halfNodeSize, halfNodeSize, halfNodeSize)
      x += halfNodeSize;
      y += halfNodeSize;
      z += halfNodeSize;

      return (x * x + y * y + z * z) < squareRadius;
   }

   /** Test if search ball S(q,r) contains node
    * <p>
    * From Fast radius neighbor search with an Octree: <a href="https://github.com/jbehley/octree"> GitHub repo</a>, 
    * <a href="http://jbehley.github.io/papers/behley2015icra.pdf"> ICRA Paper </a>.
    *
    * @param x coordinate of the query point q
    * @param y coordinate of the query point q
    * @param z coordinate of the query point q
    * @param squareRadius "squared" radius r
    * @param key address of the node
    *
    * @return true, if search ball overlaps with node, false otherwise.
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> boolean contains(NODE node, double squareRadius, double x, double y, double z)
   {
      // we exploit the symmetry to reduce the test to test whether the farthest corner is inside the search ball.
      x = Math.abs(x - node.getX());
      y = Math.abs(y - node.getY());
      z = Math.abs(z - node.getZ());

      double halfNodeSize = 0.5 * node.getSize();

      // reminder: (x, y, z) - (-halfNodeSize, -halfNodeSize, -halfNodeSize) = (x, y, z) + (halfNodeSize, halfNodeSize, halfNodeSize)
      x += halfNodeSize;
      y += halfNodeSize;
      z += halfNodeSize;

      return (x * x + y * y + z * z) < squareRadius;
   }

   /** Test if search ball S(q,r) is completely inside node.
    * <p>
    * From Fast radius neighbor search with an Octree: <a href="https://github.com/jbehley/octree"> GitHub repo</a>, 
    * <a href="http://jbehley.github.io/papers/behley2015icra.pdf"> ICRA Paper </a>.
    *
    * @param x coordinate of the query point q
    * @param y coordinate of the query point q
    * @param z coordinate of the query point q
    * @param radius  radius r
    * @param key address of the node
    * @param depth current depth in the tree
    * @param resolution resolution of the tree
    * @param treeDepth maximum depth of the tree
    *
    * @return true, if search ball is completely inside the node, false otherwise.
    */
   public static boolean inside(double x, double y, double z, double radius, OcTreeKeyReadOnly key, int depth, double resolution, int treeDepth)
   {
      double xNode = OcTreeKeyConversionTools.keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      double yNode = OcTreeKeyConversionTools.keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      double zNode = OcTreeKeyConversionTools.keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);

      return inside(x, y, z, radius, xNode, yNode, zNode, depth, resolution, treeDepth);
   }

   /** Test if search ball S(q,r) is completely inside node.
    * <p>
    * From Fast radius neighbor search with an Octree: <a href="https://github.com/jbehley/octree"> GitHub repo</a>, 
    * <a href="http://jbehley.github.io/papers/behley2015icra.pdf"> ICRA Paper </a>.
    *
    * @param x coordinate of the query point q
    * @param y coordinate of the query point q
    * @param z coordinate of the query point q
    * @param radius  radius r
    * @param xNode x coordinate of the node
    * @param yNode y coordinate of the node
    * @param zNode z coordinate of the node
    * @param depth current depth in the tree
    * @param resolution resolution of the tree
    * @param treeDepth maximum depth of the tree
    *
    * @return true, if search ball is completely inside the node, false otherwise.
    */
   public static boolean inside(double x, double y, double z, double radius, double xNode, double yNode, double zNode, int depth, double resolution,
         int treeDepth)
   {
      // we exploit the symmetry to reduce the test to test
      // whether the farthest corner is inside the search ball.
      x = Math.abs(x - xNode);
      y = Math.abs(y - yNode);
      z = Math.abs(z - zNode);

      x += radius;
      y += radius;
      z += radius;

      double halfNodeSize = 0.5 * OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);

      if (x > halfNodeSize)
         return false;
      if (y > halfNodeSize)
         return false;
      if (z > halfNodeSize)
         return false;

      return true;
   }

   /** Test if search ball S(q,r) is completely inside node.
    * <p>
    * From Fast radius neighbor search with an Octree: <a href="https://github.com/jbehley/octree"> GitHub repo</a>, 
    * <a href="http://jbehley.github.io/papers/behley2015icra.pdf"> ICRA Paper </a>.
    *
    * @param x coordinate of the query point q
    * @param y coordinate of the query point q
    * @param z coordinate of the query point q
    * @param radius  radius r
    * @param key address of the node
    *
    * @return true, if search ball is completely inside the node, false otherwise.
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> boolean inside(NODE node, double x, double y, double z, double radius)
   {
      // we exploit the symmetry to reduce the test to test
      // whether the farthest corner is inside the search ball.
      x = Math.abs(x - node.getX());
      y = Math.abs(y - node.getY());
      z = Math.abs(z - node.getZ());

      x += radius;
      y += radius;
      z += radius;

      double halfNodeSize = 0.5 * node.getSize();

      if (x > halfNodeSize)
         return false;
      if (y > halfNodeSize)
         return false;
      if (z > halfNodeSize)
         return false;

      return true;
   }
}
