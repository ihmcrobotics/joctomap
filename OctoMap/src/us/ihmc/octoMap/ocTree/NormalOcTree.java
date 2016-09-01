package us.ihmc.octoMap.ocTree;

import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.stat.descriptive.moment.Variance;

import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyDeque;
import us.ihmc.octoMap.key.OcTreeKeyList;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTreeBase;
import us.ihmc.octoMap.tools.IntersectionPlaneBoxCalculator;
import us.ihmc.octoMap.tools.OcTreeKeyTools;

public class NormalOcTree extends AbstractOccupancyOcTreeBase<NormalOcTreeNode>
{
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   public NormalOcTree(double resolution)
   {
      super(resolution);
   }

   public void updateNormals()
   {
      if (root != null)
      {
         OcTreeKey rootKey = OcTreeKeyTools.getRootKey(treeDepth);
         updateNormalsRecursively(root, rootKey, 0);
      }
   }

   private void updateNormalsRecursively(NormalOcTreeNode node, OcTreeKey nodeKey, int depth)
   {
      if (depth > treeDepth)
         throw new RuntimeException("Something went wrong.");
      if (node == null)
         throw new RuntimeException("The given node is null.");

      node.resetRegionId();

      if (!isNodeOccupied(node))
      {
         node.setNormal(null);
         return;
      }

      if (node.hasAtLeastOneChild())
      {
         int childDepth = depth + 1;

         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            if (OcTreeNodeTools.nodeChildExists(node, childIndex))
            {
               OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, nodeKey, childDepth, treeDepth);
               NormalOcTreeNode childNode = OcTreeNodeTools.getNodeChild(node, childIndex);
               updateNormalsRecursively(childNode, childKey, childDepth);
            }
         }
         node.updateNormalChildren();
      }
      else
      {
         node.setNormal(computeNodeNormal2(nodeKey, depth, true));
      }

      if (node.getNormal() != null)
      {
         Point3d center = keyToCoordinate(nodeKey);
         double computeNodeSize = getNodeSize(depth);
         intersectionPlaneBoxCalculator.setCube(computeNodeSize, center);
         intersectionPlaneBoxCalculator.setPlane(center, node.getNormal());
         node.setPlane(intersectionPlaneBoxCalculator.computeIntersections());
      }
      else
      {
         node.setPlane(null);
      }
   }

   public Vector3d computeNodeNormal(OcTreeKey key, int depth, boolean unknownStatus)
   {
      NormalOcTreeNode node = search(key, depth);
      if (node == null)
         return null;

      Vector3d normal = new Vector3d();
      OcTreeKey currentKey = new OcTreeKey();
      NormalOcTreeNode currentNode;

      int keyInterval = OcTreeKeyTools.computeKeyIntervalAtDepth(depth, treeDepth);

      int[] keyOffsets = {-keyInterval, 0, keyInterval};

      for (int kxOffset : keyOffsets)
      {
         for (int kyOffset : keyOffsets)
         {
            for (int kzOffset : keyOffsets)
            {
               currentKey.setKey(0, key.getKey(0) + kxOffset);
               currentKey.setKey(1, key.getKey(1) + kyOffset);
               currentKey.setKey(2, key.getKey(2) + kzOffset);
               currentNode = search(currentKey, depth);

               boolean nodeExists = currentNode == null;
               boolean isOccupied = !nodeExists && isNodeOccupied(currentNode);
               boolean isUnknownConsideredOccupied = nodeExists && unknownStatus;

               if (isOccupied || isUnknownConsideredOccupied)
               {
                  normal.setX(normal.getX() - kxOffset);
                  normal.setY(normal.getY() - kyOffset);
                  normal.setZ(normal.getZ() - kzOffset);
               }
            }
         }
      }

      double lengthSquared = normal.lengthSquared();
      if (lengthSquared > 1.0e-3)
      {
         normal.scale(1.0 / Math.sqrt(lengthSquared));
         return normal;
      }
      else
      {
         return null;
      }
   }

   public Vector3d computeNodeNormal2(OcTreeKey key, int depth, boolean unknownStatus)
   {
      NormalOcTreeNode node = search(key, depth);
      if (node == null)
         return null;

      Vector3d normal = new Vector3d();
      NormalOcTreeNode currentNode;

      List<OcTreeKey> neighborKeys = OcTreeKeyTools.computeNeighborKeys(key, depth, resolution, treeDepth, 0.10);

      for (int i = 0; i < neighborKeys.size(); i++)
      {
         OcTreeKey currentKey = neighborKeys.get(i);
         currentNode = search(currentKey, depth);

         boolean nodeDoesNotExists = currentNode == null;
         boolean isOccupied = !nodeDoesNotExists && isNodeOccupied(currentNode);
//         boolean isFree = !nodeDoesNotExists && !isNodeOccupied(currentNode);
         boolean isUnknownConsideredOccupied = nodeDoesNotExists && unknownStatus;

         if (isOccupied || isUnknownConsideredOccupied)
         {
            normal.setX(normal.getX() + Math.signum(key.getKey(0) - currentKey.getKey(0)));
            normal.setY(normal.getY() + Math.signum(key.getKey(1) - currentKey.getKey(1)));
            normal.setZ(normal.getZ() + Math.signum(key.getKey(2) - currentKey.getKey(2)));
         }
//         else if (isFree)
//         {
//            normal.setX(normal.getX() - Math.signum(key.getKey(0) - currentKey.getKey(0)));
//            normal.setY(normal.getY() - Math.signum(key.getKey(1) - currentKey.getKey(1)));
//            normal.setZ(normal.getZ() - Math.signum(key.getKey(2) - currentKey.getKey(2)));
//         }
      }

      double lengthSquared = normal.lengthSquared();
      if (lengthSquared > 1.0e-3)
      {
         normal.scale(1.0 / Math.sqrt(lengthSquared));
         return normal;
      }
      else
      {
         return null;
      }
   }

   public double computeNodeNeighborNormalDifference(OcTreeKey key, int depth)
   {
      NormalOcTreeNode node = search(key, depth);

      if (node == null || !node.isNormalSet())
         return Double.NaN;

      Vector3d normal = node.getNormal();
      Vector3d meanNormal = new Vector3d();
      OcTreeKey currentKey = new OcTreeKey();
      NormalOcTreeNode currentNode;

      int count = 0;

      double meanDifference = 0.0;
      Variance var = new Variance();
      int keyInterval = OcTreeKeyTools.computeKeyIntervalAtDepth(depth, treeDepth);

      int[] keyOffsets = {-keyInterval, 0, keyInterval};

      for (int kxOffset : keyOffsets)
      {
         for (int kyOffset : keyOffsets)
         {
            for (int kzOffset : keyOffsets)
            {
               if (kxOffset == 0 && kyOffset == 0 && kzOffset == 0)
                  continue;

               currentKey.setKey(0, key.getKey(0) + kxOffset);
               currentKey.setKey(1, key.getKey(1) + kyOffset);
               currentKey.setKey(2, key.getKey(2) + kzOffset);
               currentNode = search(currentKey, depth);

               if (currentNode == null || !currentNode.isNormalSet())
                  continue;

               meanNormal.add(currentNode.getNormal());
               meanDifference += 1.0 - Math.abs(currentNode.getNormal().dot(normal));
               var.increment(1.0 - Math.abs(currentNode.getNormal().dot(normal)));
               //               meanDifference = Math.max(meanDifference, 1.0 - Math.abs(currentNode.getNormal().dot(normal)));
               count++;
            }
         }
      }

      if (count == 0)
         return Double.NaN;

      meanNormal.scale(1.0 / count);
      meanDifference = 1.0 - Math.abs(meanNormal.dot(normal));

      //      meanDifference /= count;
      return meanDifference;
      //      return var.getResult();
   }

   public void updatePlanarRegionSegmentation()
   {
      double exploringRadius = 0.08;
      double maxMistanceFromPlane = 0.07;
      double angleThreshold = Math.toRadians(6.0);
      int depth = treeDepth;
      Random random = new Random(45561L);

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable(treeDepth))
      {
         NormalOcTreeNode node = superNode.getNode();

         if (node.isPartOfRegion() || !node.isNormalSet())
            continue;

         OcTreeKey nodeKey = superNode.getKey();
         int regionId = random.nextInt(Integer.MAX_VALUE);
         PlanarRegion planarRegion = new PlanarRegion(regionId);
         planarRegion.update(node.getNormal(), keyToCoordinate(nodeKey, depth));
         node.setRegionId(planarRegion.getId());

         //         growPlanarRegionRecursively(planarRegion, nodeKey, depth, exploringRadius, maxMistanceFromPlane, angleThreshold);
         growPlanarRegionIteratively(planarRegion, nodeKey, depth, exploringRadius, maxMistanceFromPlane, angleThreshold);
      }
   }

   private final OcTreeKeyList tempNeighborKeys = new OcTreeKeyList();

   private void growPlanarRegionIteratively(PlanarRegion planarRegion, OcTreeKey nodeKey, int depth, double searchRadius, double maxMistanceFromPlane,
         double angleThreshold)
   {
      HashSet<OcTreeKey> exploredKeys = new HashSet<>();
      exploredKeys.add(nodeKey);

      OcTreeKeyDeque keysToExplore = new OcTreeKeyDeque();
      OcTreeKeyTools.computeNeighborKeys(nodeKey, depth, resolution, treeDepth, searchRadius, tempNeighborKeys);
      keysToExplore.addAll(tempNeighborKeys);
      keysToExplore.removeAll(exploredKeys);

      while (!keysToExplore.isEmpty())
      {
         OcTreeKey currentKey = keysToExplore.poll();
         exploredKeys.add(currentKey);

         NormalOcTreeNode currentNode = search(currentKey, depth);
         if (currentNode == null || !currentNode.isNormalSet() || currentNode.isPartOfRegion() || !isNodeOccupied(currentNode))
            continue;
         Vector3d currentNodeNormal = currentNode.getNormal();
         Point3d currentNodeCenter = keyToCoordinate(currentKey, depth);
         if (planarRegion.absoluteDistance(currentNodeCenter) < maxMistanceFromPlane && planarRegion.absoluteAngle(currentNodeNormal) < angleThreshold)
         {
            planarRegion.update(currentNodeNormal, currentNodeCenter);
            currentNode.setRegionId(planarRegion.getId());
            OcTreeKeyTools.computeNeighborKeys(currentKey, depth, resolution, treeDepth, searchRadius, tempNeighborKeys);
            keysToExplore.addAll(tempNeighborKeys);
            keysToExplore.removeAll(exploredKeys);
         }
      }
   }

   @Override
   protected NormalOcTreeNode createRootNode()
   {
      return new NormalOcTreeNode();
   }
}
