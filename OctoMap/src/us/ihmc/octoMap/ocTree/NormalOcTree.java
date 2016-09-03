package us.ihmc.octoMap.ocTree;

import java.util.HashSet;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.stat.descriptive.moment.Variance;

import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyDeque;
import us.ihmc.octoMap.key.OcTreeKeyList;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTreeBase;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.octoMap.pointCloud.SweepCollection;
import us.ihmc.octoMap.tools.OcTreeKeyTools;

public class NormalOcTree extends AbstractOccupancyOcTreeBase<NormalOcTreeNode>
{
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

   private final Vector3d tempNormal = new Vector3d();

   private void updateNormalsRecursively(NormalOcTreeNode node, OcTreeKeyReadOnly nodeKey, int depth)
   {
      if (depth > treeDepth)
         throw new RuntimeException("Something went wrong.");
      if (node == null)
         throw new RuntimeException("The given node is null.");

      node.resetRegionId();

      if (!isNodeOccupied(node))
      {
         node.resetNormal();
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
         if (computeNodeNormal2(nodeKey, depth, true, tempNormal))
            node.setNormal(tempNormal);
         else
            node.resetNormal();
      }
   }

   public Vector3d computeNodeNormal(OcTreeKeyReadOnly key, int depth, boolean unknownStatus)
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

   private final OcTreeKeyList tempNeighborKeysForNormal = new OcTreeKeyList();

   public Vector3d computeNodeNormal2(OcTreeKeyReadOnly key, int depth, boolean unknownStatus)
   {
      Vector3d normal = new Vector3d();
      boolean success = computeNodeNormal2(key, depth, unknownStatus, normal);
      return success ? normal : null;
   }

   public boolean computeNodeNormal2(OcTreeKeyReadOnly key, int depth, boolean unknownStatus, Vector3d normalToPack)
   {
      NormalOcTreeNode node = search(key, depth);
      if (node == null)
         return false;

      NormalOcTreeNode currentNode;

      OcTreeKeyTools.computeNeighborKeys(key, depth, resolution, treeDepth, 0.10, tempNeighborKeysForNormal);

      for (int i = 0; i < tempNeighborKeysForNormal.size(); i++)
      {
         OcTreeKey currentKey = tempNeighborKeysForNormal.get(i);
         currentNode = search(currentKey, depth);

         boolean nodeDoesNotExists = currentNode == null;
         boolean isOccupied = !nodeDoesNotExists && isNodeOccupied(currentNode);
//         boolean isFree = !nodeDoesNotExists && !isNodeOccupied(currentNode);
         boolean isUnknownConsideredOccupied = nodeDoesNotExists && unknownStatus;

         if (isOccupied || isUnknownConsideredOccupied)
         {
            normalToPack.setX(normalToPack.getX() + Math.signum(key.getKey(0) - currentKey.getKey(0)));
            normalToPack.setY(normalToPack.getY() + Math.signum(key.getKey(1) - currentKey.getKey(1)));
            normalToPack.setZ(normalToPack.getZ() + Math.signum(key.getKey(2) - currentKey.getKey(2)));
         }
//         else if (isFree)
//         {
//            normalToPack.setX(normalToPack.getX() - Math.signum(key.getKey(0) - currentKey.getKey(0)));
//            normalToPack.setY(normalToPack.getY() - Math.signum(key.getKey(1) - currentKey.getKey(1)));
//            normalToPack.setZ(normalToPack.getZ() - Math.signum(key.getKey(2) - currentKey.getKey(2)));
//         }
      }

      double lengthSquared = normalToPack.lengthSquared();
      if (lengthSquared > 1.0e-3)
      {
         normalToPack.scale(1.0 / Math.sqrt(lengthSquared));
         return true;
      }
      else
      {
         return false;
      }
   }

   public double computeNodeNeighborNormalDifference(OcTreeKeyReadOnly key, int depth)
   {
      NormalOcTreeNode node = search(key, depth);

      if (node == null || !node.isNormalSet())
         return Double.NaN;

      Vector3d normal = new Vector3d();
      node.getNormal(normal);
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

               Vector3d currentNodeNormal = new Vector3d();
               currentNode.getNormal(currentNodeNormal);
               meanNormal.add(currentNodeNormal);
               double dot = currentNodeNormal.dot(normal);
               meanDifference += 1.0 - Math.abs(dot);
               var.increment(1.0 - Math.abs(dot));
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
      Vector3d nodeNormal = new Vector3d();

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable(treeDepth))
      {
         NormalOcTreeNode node = superNode.getNode();

         if (node.isPartOfRegion() || !node.isNormalSet())
            continue;

         OcTreeKeyReadOnly nodeKey = superNode.getKey();
         int regionId = random.nextInt(Integer.MAX_VALUE);
         PlanarRegion planarRegion = new PlanarRegion(regionId);
         node.getNormal(nodeNormal);
         planarRegion.update(nodeNormal, keyToCoordinate(nodeKey, depth));
         node.setRegionId(planarRegion.getId());

         //         growPlanarRegionRecursively(planarRegion, nodeKey, depth, exploringRadius, maxMistanceFromPlane, angleThreshold);
         growPlanarRegionIteratively(planarRegion, nodeKey, depth, exploringRadius, maxMistanceFromPlane, angleThreshold);
      }
   }

   private final OcTreeKeyList tempNeighborKeysForPlanarRegion = new OcTreeKeyList();

   private void growPlanarRegionIteratively(PlanarRegion planarRegion, OcTreeKeyReadOnly nodeKey, int depth, double searchRadius, double maxMistanceFromPlane,
         double angleThreshold)
   {
      HashSet<OcTreeKeyReadOnly> exploredKeys = new HashSet<>();
      exploredKeys.add(nodeKey);

      OcTreeKeyDeque keysToExplore = new OcTreeKeyDeque();
      OcTreeKeyTools.computeNeighborKeys(nodeKey, depth, resolution, treeDepth, searchRadius, tempNeighborKeysForPlanarRegion);
      keysToExplore.addAll(tempNeighborKeysForPlanarRegion);
      keysToExplore.removeAll(exploredKeys);
      Vector3d currentNodeNormal = new Vector3d();

      while (!keysToExplore.isEmpty())
      {
         OcTreeKey currentKey = keysToExplore.poll();
         exploredKeys.add(currentKey);

         NormalOcTreeNode currentNode = search(currentKey, depth);
         if (currentNode == null || !currentNode.isNormalSet() || currentNode.isPartOfRegion() || !isNodeOccupied(currentNode))
            continue;
         currentNode.getNormal(currentNodeNormal);
         Point3d currentNodeCenter = keyToCoordinate(currentKey, depth);
         if (planarRegion.absoluteDistance(currentNodeCenter) < maxMistanceFromPlane && planarRegion.absoluteAngle(currentNodeNormal) < angleThreshold)
         {
            planarRegion.update(currentNodeNormal, currentNodeCenter);
            currentNode.setRegionId(planarRegion.getId());
            OcTreeKeyTools.computeNeighborKeys(currentKey, depth, resolution, treeDepth, searchRadius, tempNeighborKeysForPlanarRegion);
            keysToExplore.addAll(tempNeighborKeysForPlanarRegion);
            keysToExplore.removeAll(exploredKeys);
         }
      }
   }


   public void updateSweepCollectionHitLocations(SweepCollection sweepCollection, double alphaUpdate, boolean lazyEvaluation)
   {
      for (int i = 0; i < sweepCollection.getNumberOfSweeps(); i ++)
         updateHitLocations(sweepCollection.getSweep(i), alphaUpdate, lazyEvaluation);
   }

   public void updateHitLocations(PointCloud pointCloud, double alphaUpdate, boolean lazyEvaluation)
   {
      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point3f point = pointCloud.getPoint(i);
         OcTreeKey key = coordinateToKey(point);
         updateNodeCenter(root, key, 0, point, alphaUpdate, lazyEvaluation);
      }
   }

   private void updateNodeCenter(NormalOcTreeNode node, OcTreeKeyReadOnly key, int depth, Point3f centerUpdate, double alphaUpdate, boolean lazyEvaluation)
   {
      if (depth < treeDepth)
      {
         int childIndex = OcTreeKeyTools.computeChildIndex(key, treeDepth - 1 - depth);
         NormalOcTreeNode child;
         if (node.hasArrayForChildren() && (child = node.getChildUnsafe(childIndex)) != null)
         {
            updateNodeCenter(child, key, depth + 1, centerUpdate, alphaUpdate, lazyEvaluation);
            
            if (!lazyEvaluation)
               node.updateCenterChildren();
            return;
         }
         else
         {
            node.updateCenter(centerUpdate, alphaUpdate);
            return;
         }
      }
      else
      {
         node.updateCenter(centerUpdate, alphaUpdate);
         return;
      }
   }

   @Override
   protected NormalOcTreeNode createEmptyNode()
   {
      return new NormalOcTreeNode();
   }
}
