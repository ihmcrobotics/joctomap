package us.ihmc.octoMap.ocTree;

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
import us.ihmc.octoMap.key.OcTreeKeySet;
import us.ihmc.octoMap.node.NormalOcTreeNode;
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

   private void updateNormalsRecursively(NormalOcTreeNode node, OcTreeKeyReadOnly nodeKey, int depth)
   {
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
            NormalOcTreeNode childNode;
            if ((childNode = node.getChildUnsafe(childIndex)) != null)
            {
               OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, nodeKey, childDepth, treeDepth);
               updateNormalsRecursively(childNode, childKey, childDepth);
            }
         }
         node.updateNormalChildren();
      }
      else
      {
//         computeNodeNormalWithSphericalNeighborhood(nodeKey, depth, true);
         computeNodeNormalRansac(nodeKey, depth, true);
      }
   }

   public void computeNodeNormalWithDirectNeighbors(OcTreeKeyReadOnly key, int depth, boolean unknownStatus)
   {
      NormalOcTreeNode node = search(key, depth);
      if (node == null)
         return;

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
         node.setNormal(normal);
      }
      else
      {
         node.resetNormal();
      }
   }

   private final OcTreeKeyList tempNeighborKeysForNormal = new OcTreeKeyList();
   private final Vector3d normal = new Vector3d();
   private final Vector3d nodeCenterToNeighborCenter = new Vector3d();
   private final Point3d nodeCenter = new Point3d();
   private final Point3d neighborCenter = new Point3d();

   public void computeNodeNormalWithSphericalNeighborhood(OcTreeKeyReadOnly key, int depth, boolean unknownStatus)
   {
      NormalOcTreeNode node = search(key, depth);
      if (node == null)
         return;

      if (node.isCenterSet())
         node.getCenter(nodeCenter);
      else
         keyToCoordinate(key, nodeCenter, depth);

      NormalOcTreeNode currentNode;

      OcTreeKeyTools.computeNeighborKeys(key, depth, resolution, treeDepth, 0.10, tempNeighborKeysForNormal);
      normal.set(0.0, 0.0, 0.0);

      for (int i = tempNeighborKeysForNormal.size() - 1; i >= 0 ; i--)
      {
         OcTreeKey currentKey = tempNeighborKeysForNormal.unsafeGet(i);
         currentNode = search(currentKey, depth);

         if (currentNode == null)
         {
            if (unknownStatus)
            {
               normal.setX(normal.getX() + Math.signum(key.getKey(0) - currentKey.getKey(0)));
               normal.setY(normal.getY() + Math.signum(key.getKey(1) - currentKey.getKey(1)));
               normal.setZ(normal.getZ() + Math.signum(key.getKey(2) - currentKey.getKey(2)));
            }
            tempNeighborKeysForNormal.fastRemove(i);
         }
         else if (isNodeOccupied(currentNode))
         {
            normal.setX(normal.getX() + Math.signum(key.getKey(0) - currentKey.getKey(0)));
            normal.setY(normal.getY() + Math.signum(key.getKey(1) - currentKey.getKey(1)));
            normal.setZ(normal.getZ() + Math.signum(key.getKey(2) - currentKey.getKey(2)));
         }
      }

      double lengthSquared = normal.lengthSquared();
      if (lengthSquared > 1.0e-3)
      {
         normal.scale(1.0 / Math.sqrt(lengthSquared));
         node.setNormal(normal);

         float normalQuality = 0.0f;
         int numberOfPoints = 0;

         for (int i = 0; i < tempNeighborKeysForNormal.size(); i++)
         {
            OcTreeKey currentKey = tempNeighborKeysForNormal.unsafeGet(i);
            currentNode = search(currentKey, depth);

            if (currentNode != null && currentNode.isCenterSet())
            {
               currentNode.getCenter(neighborCenter);
               nodeCenterToNeighborCenter.sub(nodeCenter, neighborCenter);
               normalQuality += Math.abs(normal.dot(nodeCenterToNeighborCenter));
               numberOfPoints++;
            }
         }

         if (numberOfPoints == 0)
            normalQuality = Float.POSITIVE_INFINITY;
         else
            normalQuality /= numberOfPoints;
         node.setNormalQuality(normalQuality);
      }
      else
      {
         node.resetNormal();
      }
   }

   private final Random random = new Random(1651L);
   private final Vector3d normalCandidate = new Vector3d();
   private final Point3d[] randomDraw = {new Point3d(), new Point3d(), new Point3d()};

   public void computeNodeNormalRansac(OcTreeKeyReadOnly key, int depth, boolean unknownStatus)
   {
      NormalOcTreeNode node = search(key, depth);
      if (node == null)
         return;

      if (!node.isCenterSet() || !node.isNormalQualitySet())
      {
         computeNodeNormalWithSphericalNeighborhood(key, depth, unknownStatus);
         return;
      }

      node.getCenter(nodeCenter);

      NormalOcTreeNode currentNode;

      OcTreeKeyTools.computeNeighborKeys(key, depth, resolution, treeDepth, 0.10, tempNeighborKeysForNormal);
      normalCandidate.set(0.0, 0.0, 0.0);

      int index = 0;

      while (index < 3)
      {
         // Did not find three points, just fall back to the naive way
         if (tempNeighborKeysForNormal.isEmpty())
         {
            computeNodeNormalWithSphericalNeighborhood(key, depth, unknownStatus);
            return;
         }

         int nextInt = random.nextInt(tempNeighborKeysForNormal.size());
         OcTreeKey currentKey = tempNeighborKeysForNormal.unsafeGet(nextInt);
         currentNode = search(currentKey, depth);
         
         if (currentNode != null && currentNode.isCenterSet())
         {
            currentNode.getCenter(randomDraw[index++]);
         }
         tempNeighborKeysForNormal.fastRemove(nextInt);
      }

      double v1_x = randomDraw[1].getX() - randomDraw[0].getX();
      double v1_y = randomDraw[1].getY() - randomDraw[0].getY();
      double v1_z = randomDraw[1].getZ() - randomDraw[0].getZ();

      double v2_x = randomDraw[2].getX() - randomDraw[0].getX();
      double v2_y = randomDraw[2].getY() - randomDraw[0].getY();
      double v2_z = randomDraw[2].getZ() - randomDraw[0].getZ();

      normalCandidate.setX(v1_y * v2_z - v1_z * v2_y);
      normalCandidate.setY(v2_x * v1_z - v2_z * v1_x);
      normalCandidate.setZ(v1_x * v2_y - v1_y * v2_x);
      normalCandidate.normalize();


      float normalQuality = 0.0f;
      int numberOfPoints = 3; // The three points picked randomly are on the plane

      for (int i = 0; i < tempNeighborKeysForNormal.size(); i++)
      {
         OcTreeKey currentKey = tempNeighborKeysForNormal.unsafeGet(i);
         currentNode = search(currentKey, depth);

         if (currentNode != null && currentNode.isCenterSet())
         {
            currentNode.getCenter(neighborCenter);
            nodeCenterToNeighborCenter.sub(nodeCenter, neighborCenter);
            normalQuality += Math.abs(normalCandidate.dot(nodeCenterToNeighborCenter));
            numberOfPoints++;
         }
      }

      normalQuality /= numberOfPoints;
      if (normalQuality < node.getNormalQuality())
      {
         node.getNormal(normal);
         if (normal.dot(normalCandidate) < 0.0)
            normalCandidate.negate();
         node.setNormal(normalCandidate);
         node.setNormalQuality(normalQuality);
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

   public void updatePlanarRegionSegmentation(int depth)
   {
      double exploringRadius = 2.0 * getNodeSize(depth);
      double maxMistanceFromPlane = 0.05;
      double angleThreshold = Math.toRadians(5.0);
      Random random = new Random(45561L);
      Vector3d nodeNormal = new Vector3d();

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable(treeDepth))
      {
         NormalOcTreeNode node = superNode.getNode();

         if (!isNodeOccupied(node) || node.isPartOfRegion() || !node.isNormalSet())
            continue;

         OcTreeKeyReadOnly nodeKey = superNode.getKey();
         int regionId = random.nextInt(Integer.MAX_VALUE);
         PlanarRegion planarRegion = new PlanarRegion(regionId);
         node.getNormal(nodeNormal);
         planarRegion.update(nodeNormal, keyToCoordinate(nodeKey, depth));
         node.setRegionId(planarRegion.getId());

         growPlanarRegionIteratively(planarRegion, nodeKey, depth, exploringRadius, maxMistanceFromPlane, angleThreshold);
      }
   }

   private final OcTreeKeyList tempNeighborKeysForPlanarRegion = new OcTreeKeyList();
   private final Vector3d normalCandidateToCurrentRegion = new Vector3d();
   private final Point3d centerCandidateToCurrentRegion = new Point3d();
   private final OcTreeKeySet exploredKeys = new OcTreeKeySet();
   private final OcTreeKeyDeque keysToExplore = new OcTreeKeyDeque();

   private void growPlanarRegionIteratively(PlanarRegion planarRegion, OcTreeKeyReadOnly nodeKey, int depth, double searchRadius, double maxMistanceFromPlane,
         double angleThreshold)
   {
      exploredKeys.clear();
      keysToExplore.clear();

      exploredKeys.add(nodeKey);

      OcTreeKeyTools.computeNeighborKeys(nodeKey, depth, resolution, treeDepth, searchRadius, tempNeighborKeysForPlanarRegion);
      keysToExplore.addAll(tempNeighborKeysForPlanarRegion);
      keysToExplore.removeAll(exploredKeys);
      

      while (!keysToExplore.isEmpty())
      {
         OcTreeKey currentKey = keysToExplore.poll();
         exploredKeys.add(currentKey);

         NormalOcTreeNode currentNode = search(currentKey, depth);
         if (currentNode == null || !currentNode.isNormalSet() || !currentNode.isCenterSet() || currentNode.isPartOfRegion() || !isNodeOccupied(currentNode))
            continue;

         currentNode.getNormal(normalCandidateToCurrentRegion);
         currentNode.getCenter(centerCandidateToCurrentRegion);

         if (planarRegion.absoluteDistance(centerCandidateToCurrentRegion) < maxMistanceFromPlane && planarRegion.absoluteAngle(normalCandidateToCurrentRegion) < angleThreshold)
         {
            planarRegion.update(normalCandidateToCurrentRegion, centerCandidateToCurrentRegion);
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
         if (useBoundingBoxLimit && !isInBoundingBox(point))
            continue;
         OcTreeKey key = coordinateToKey(point);
         updateNodeCenterRecursively(root, key, 0, point, alphaUpdate, lazyEvaluation);
      }
   }

   private void updateNodeCenterRecursively(NormalOcTreeNode node, OcTreeKeyReadOnly key, int depth, Point3f centerUpdate, double alphaUpdate, boolean lazyEvaluation)
   {
      if (depth < treeDepth)
      {
         int childIndex = OcTreeKeyTools.computeChildIndex(key, treeDepth - 1 - depth);
         NormalOcTreeNode child;
         if (node.hasArrayForChildren() && (child = node.getChildUnsafe(childIndex)) != null)
         {
            node.updateCenter(centerUpdate, alphaUpdate);
            updateNodeCenterRecursively(child, key, depth + 1, centerUpdate, alphaUpdate, lazyEvaluation);
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
