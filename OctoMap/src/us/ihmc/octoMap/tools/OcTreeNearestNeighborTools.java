package us.ihmc.octoMap.tools;

import static us.ihmc.octoMap.tools.OcTreeKeyConversionTools.*;

import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyList;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOcTreeNode;

public class OcTreeNearestNeighborTools
{
   /**
    * Radius neighbor queries where radius determines the maximal radius of reported indices of points in radiusNeighborKeysToPack.
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> void radiusNeighbors(NODE rootNode, Point3d query, double radius,
         OcTreeKeyList radiusNeighborKeysToPack, List<NODE> radiusNeighborsToPack, double resolution, int treeDepth)
   {
      radiusNeighbors(rootNode, query.getX(), query.getY(), query.getZ(), radius, radiusNeighborKeysToPack, radiusNeighborsToPack, resolution, treeDepth);
   }

   /**
    * Radius neighbor queries where radius determines the maximal radius of reported indices of points in radiusNeighborKeysToPack.
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> void radiusNeighbors(NODE rootNode, double x, double y, double z, double radius,
         OcTreeKeyList radiusNeighborKeysToPack, List<NODE> radiusNeighborsToPack, double resolution, int treeDepth)
   {
      OcTreeKeyReadOnly rootKey = OcTreeKeyTools.getRootKey(treeDepth);
      double radiusSquared = radius * radius;
      radiusNeighbors(rootNode, rootKey, 0.0, 0.0, 0.0, x, y, z, radius, radiusSquared, radiusNeighborKeysToPack, radiusNeighborsToPack, 0, resolution,
            treeDepth);
   }

   private static <NODE extends AbstractOcTreeNode<NODE>> void radiusNeighbors(NODE node, OcTreeKeyReadOnly nodeKey, double xNode, double yNode, double zNode,
         double x, double y, double z, double radius, double radiusSquared, OcTreeKeyList radiusNeighborKeysToPack, List<NODE> radiusNeighborsToPack, int depth,
         double resolution, int treeDepth)
   {

      // if search ball S(q,r) contains octant, simply add point indexes.
      if (contains(x, y, z, radiusSquared, xNode, yNode, zNode, depth, resolution, treeDepth))
      {
         if (radiusNeighborKeysToPack == null)
            addLeafNodesRecursively(node, radiusNeighborsToPack, depth, treeDepth);
         else if (radiusNeighborsToPack == null)
            addLeafKeysRecursively(nodeKey, node, radiusNeighborKeysToPack, depth, treeDepth);
         else
            addLeafNodesAndKeysRecursively(nodeKey, node, radiusNeighborKeysToPack, radiusNeighborsToPack, depth, treeDepth);

         return; // early pruning.
      }

      if (!node.hasAtLeastOneChild())
      {
         double dx = x - xNode;
         double dy = y - yNode;
         double dz = z - zNode;

         double distanceSquared = dx * dx + dy * dy + dz * dz;
         if (distanceSquared < radiusSquared)
         {
            if (radiusNeighborKeysToPack != null)
               radiusNeighborKeysToPack.add(nodeKey);
            if (radiusNeighborsToPack != null)
               radiusNeighborsToPack.add(node);
         }

         return;
      }

      // check whether child nodes are in range.
      for (int childIndex = 0; childIndex < 8; childIndex++)
      {
         NODE child = node.getChildUnsafe(childIndex);
         if (child == null)
            continue;
         int childDepth = depth + 1;
         OcTreeKeyReadOnly childKey = OcTreeKeyTools.computeChildKey(childIndex, nodeKey, childDepth, treeDepth);

         double xChild = keyToCoordinate(childKey.getKey(0), childDepth, resolution, treeDepth);
         double yChild = keyToCoordinate(childKey.getKey(1), childDepth, resolution, treeDepth);
         double zChild = keyToCoordinate(childKey.getKey(2), childDepth, resolution, treeDepth);

         if (!overlaps(x, y, z, radius, radiusSquared, xChild, yChild, zChild, childDepth, resolution, treeDepth))
            continue;
         radiusNeighbors(child, childKey, xChild, yChild, zChild, x, y, z, radius, radiusSquared, radiusNeighborKeysToPack, radiusNeighborsToPack, childDepth,
               resolution, treeDepth);
      }
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> boolean findNeighbor(NODE rootNode, double x, double y, double z, double minDistance,
         double maxDistance, OcTreeKey nearestNeighborKey, int depth, double resolution, int treeDepth)
   {
      OcTreeKeyReadOnly rootKey = OcTreeKeyTools.getRootKey(treeDepth);
      return findNeighbor(rootKey, rootNode, x, y, z, minDistance, maxDistance, nearestNeighborKey, depth, resolution, treeDepth);
   }

   /** \brief nearest neighbor queries. Using minDistance >= 0, we explicitly disallow self-matches.
    * @return index of nearest neighbor n with Distance::compute(query, n) > minDistance and otherwise -1.
    **/
   private static <NODE extends AbstractOcTreeNode<NODE>> boolean findNeighbor(OcTreeKeyReadOnly key, NODE node, double x, double y, double z,
         double minDistance, double maxDistance, OcTreeKey nearestNeighborKey, int depth, double resolution, int treeDepth)
   {
      double xNode = keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      double yNode = keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      double zNode = keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);

      // 1. first descend to leaf and check in leafs points.
      if (!node.hasAtLeastOneChild())
      {
         double sqrMaxDistance = maxDistance * maxDistance;
         double sqrMinDistance = (minDistance < 0) ? minDistance : minDistance * minDistance;

         double dx = x - xNode;
         double dy = y - xNode;
         double dz = z - xNode;

         double distanceSquared = dx * dx + dy * dy + dz * dz;
         return distanceSquared > sqrMinDistance && distanceSquared < sqrMaxDistance;
      }

      // determine Morton code for each point...
      int mortonCode = 0;
      if (x > xNode)
         mortonCode |= 1;
      if (y > yNode)
         mortonCode |= 2;
      if (z > zNode)
         mortonCode |= 4;

      NODE child = node.getChildUnsafe(mortonCode);

      if (child != null)
      {
         int childDepth = depth + 1;
         OcTreeKey childKey = OcTreeKeyTools.computeChildKey(mortonCode, key, childDepth, treeDepth);
         if (findNeighbor(childKey, child, x, y, z, minDistance, maxDistance, nearestNeighborKey, depth, resolution, treeDepth))
            return true;
      }

      // 2. if current best point completely inside, just return.
      double sqrMaxDistance = maxDistance * maxDistance;

      // 3. check adjacent octants for overlap and check these if necessary.
      for (int childIndex = 0; childIndex < 8; childIndex++)
      {
         if (childIndex == mortonCode)
            continue;

         child = node.getChildUnsafe(childIndex);

         if (child == null)
            continue;

         int childDepth = depth + 1;
         OcTreeKeyReadOnly childKey = OcTreeKeyTools.computeChildKey(childIndex, key, childDepth, treeDepth);

         if (!overlaps(x, y, z, maxDistance, sqrMaxDistance, childKey, childDepth, resolution, treeDepth))
            continue;

         if (findNeighbor(childKey, child, x, y, z, minDistance, maxDistance, nearestNeighborKey, childDepth, resolution, treeDepth))
            return true; // early pruning
      }

      // all children have been checked...check if point is inside the current octant...
      return inside(x, y, z, maxDistance, key, depth, resolution, treeDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> void addLeafNodesAndKeysRecursively(OcTreeKeyReadOnly key, NODE node, OcTreeKeyList keysToPack,
         List<NODE> nodesToPack, int depth, int treeDepth)
   {
      if (!node.hasAtLeastOneChild())
      {
         keysToPack.add(key);
         nodesToPack.add(node);
         return;
      }

      int childDepth = depth + 1;

      for (int childIndex = 0; childIndex < 8; childIndex++)
      {
         NODE childNode = node.getChildUnsafe(childIndex);
         if (childNode == null)
            continue;

         OcTreeKeyReadOnly childKey = OcTreeKeyTools.computeChildKey(childIndex, key, childDepth, treeDepth);
         addLeafNodesAndKeysRecursively(childKey, childNode, keysToPack, nodesToPack, childDepth, treeDepth);
      }
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> void addLeafNodesRecursively(NODE node, List<NODE> nodesToPack, int depth, int treeDepth)
   {
      if (!node.hasAtLeastOneChild())
      {
         nodesToPack.add(node);
         return;
      }

      int childDepth = depth + 1;

      for (int childIndex = 0; childIndex < 8; childIndex++)
      {
         NODE childNode = node.getChildUnsafe(childIndex);
         if (childNode == null)
            continue;

         addLeafNodesRecursively(childNode, nodesToPack, childDepth, treeDepth);
      }
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> void addLeafKeysRecursively(OcTreeKeyReadOnly key, NODE node, OcTreeKeyList keysToPack, int depth,
         int treeDepth)
   {
      if (!node.hasAtLeastOneChild())
      {
         keysToPack.add(key);
         return;
      }

      int childDepth = depth + 1;

      for (int childIndex = 0; childIndex < 8; childIndex++)
      {
         NODE childNode = node.getChildUnsafe(childIndex);
         if (childNode == null)
            continue;

         OcTreeKeyReadOnly childKey = OcTreeKeyTools.computeChildKey(childIndex, key, childDepth, treeDepth);
         addLeafKeysRecursively(childKey, childNode, keysToPack, childDepth, treeDepth);
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
    *
    * @return true, if search ball overlaps with node, false otherwise.
    */
   public static boolean overlaps(double x, double y, double z, double radius, double squareRadius, OcTreeKeyReadOnly key, int depth, double resolution,
         int treeDepth)
   {
      double xNode = keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      double yNode = keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      double zNode = keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);

      return overlaps(x, y, z, radius, squareRadius, xNode, yNode, zNode, depth, resolution, treeDepth);
   }

   public static boolean overlaps(double x, double y, double z, double radius, double squareRadius, double xNode, double yNode, double zNode, int depth,
         double resolution, int treeDepth)
   {
      // we exploit the symmetry to reduce the test to testing if its inside the Minkowski sum around the positive quadrant.
      double dx = Math.abs(x - xNode);
      double dy = Math.abs(y - yNode);
      double dz = Math.abs(z - zNode);

      double halfNodeSize = 0.5 * computeNodeSize(depth, resolution, treeDepth);

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
    * @param key address of the node
    *
    * @return true, if search ball overlaps with node, false otherwise.
    */
   public static boolean contains(double x, double y, double z, double squareRadius, OcTreeKeyReadOnly key, int depth, double resolution, int treeDepth)
   {
      // we exploit the symmetry to reduce the test to test whether the farthest corner is inside the search ball.
      double xNode = keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      double yNode = keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      double zNode = keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);
      return contains(x, y, z, squareRadius, xNode, yNode, zNode, depth, resolution, treeDepth);
   }

   public static boolean contains(double x, double y, double z, double squareRadius, double xNode, double yNode, double zNode, int depth, double resolution,
         int treeDepth)
   {
      // we exploit the symmetry to reduce the test to test whether the farthest corner is inside the search ball.
      x = Math.abs(x - xNode);
      y = Math.abs(y - yNode);
      z = Math.abs(z - zNode);

      double halfNodeSize = 0.5 * computeNodeSize(depth, resolution, treeDepth);

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
    *
    * @return true, if search ball is completely inside the node, false otherwise.
    */
   public static boolean inside(double x, double y, double z, double radius, OcTreeKeyReadOnly key, int depth, double resolution, int treeDepth)
   {
      double xNode = keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      double yNode = keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      double zNode = keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);

      return inside(x, y, z, radius, xNode, yNode, zNode, depth, resolution, treeDepth);
   }

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

      double halfNodeSize = 0.5 * computeNodeSize(depth, resolution, treeDepth);

      if (x > halfNodeSize)
         return false;
      if (y > halfNodeSize)
         return false;
      if (z > halfNodeSize)
         return false;

      return true;
   }
}
