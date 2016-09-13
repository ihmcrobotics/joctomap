package us.ihmc.octoMap.ocTree;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.stat.descriptive.moment.Variance;

import gnu.trove.map.hash.TDoubleObjectHashMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.octoMap.iterators.LeafIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyDeque;
import us.ihmc.octoMap.key.OcTreeKeyList;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTree;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.octoMap.pointCloud.SweepCollection;
import us.ihmc.octoMap.tools.OcTreeKeyTools;
import us.ihmc.robotics.time.TimeTools;

public class NormalOcTree extends AbstractOccupancyOcTree<NormalOcTreeNode>
{
   private enum NormalComputationMethod {DIRECT_NEIGHBORS, CLOSE_NEIGHBORS, RANSAC};
   private static final NormalComputationMethod NORMAL_COMPUTATION_METHOD = NormalComputationMethod.RANSAC;

   private final TDoubleObjectHashMap<TIntObjectHashMap<OcTreeKeyList>> neighborOffsetsCached = new TDoubleObjectHashMap<>(4);
   private final LeafIterable<NormalOcTreeNode> leafIterable;

   public NormalOcTree(double resolution)
   {
      super(resolution);
      leafIterable = new LeafIterable<>(this, treeDepth, true);
   }

   private final TIntObjectHashMap<NormalOcTreeNode> keyToNodeMap = new TIntObjectHashMap<>();
   private final Vector3d tempInitialNormalGuess = new Vector3d();
   private final Point3d tempCenterUpdate = new Point3d();

   public void updateNodeFromSweepCollection(SweepCollection sweepCollection)
   {
      System.out.println("Entering updateNodeFromSweepCollection sweep size: " + sweepCollection.getNumberOfSweeps());
      for (int i = 0; i < sweepCollection.getNumberOfSweeps(); i++)
         System.out.println("Point cloud size: " + sweepCollection.getSweep(i).size());
      long startTime = System.nanoTime();
     
      for (int i = 0; i < sweepCollection.getNumberOfSweeps(); i++)
      {
         Point3d sensorOrigin = sweepCollection.getSweepOrigin(i);
         PointCloud scan = sweepCollection.getSweep(i);

         updateNodesFromPointCloud(sensorOrigin, scan);
      }

      long endTime = System.nanoTime();
      System.out.println("Exiting  updateNodeFromSweepCollection took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));
   }

   public void updateNodesFromPointCloud(Point3d sensorOrigin, PointCloud scan)
   {
      Point3d scanPoint = new Point3d();
      double alphaCenterUpdate = 0.1;
      double minRangeSquared = minInsertRange < 0 ? 0 : minInsertRange * minInsertRange;
      double maxRangeSquared = maxInsertRange < 0 ? Double.POSITIVE_INFINITY : maxInsertRange * maxInsertRange;

      for (int i = 0; i < scan.size(); i++)
      {
         scanPoint.set(scan.getPoint(i));
         double distanceSquared = scanPoint.distanceSquared(sensorOrigin);
         if (distanceSquared < maxRangeSquared && distanceSquared > minRangeSquared)
         {
            NormalOcTreeNode node = updateNode(scanPoint, true);
            node.updateCenter(scanPoint, alphaCenterUpdate);
            if (!node.isNormalSet())
            {
               tempCenterUpdate.set(scanPoint);
               tempInitialNormalGuess.sub(sensorOrigin, tempCenterUpdate);
               tempInitialNormalGuess.normalize();
               node.setNormal(tempInitialNormalGuess);
               node.setNormalQuality(Float.POSITIVE_INFINITY);
            }
         }
      }
   }

   public void updateNormalsAndPlanarRegions(int depth)
   {
      keyToNodeMap.clear();
      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable)
      {
         NormalOcTreeNode node = superNode.getNode();
         node.resetRegionId();
         node.resetHasBeenCandidateForRegion();
         keyToNodeMap.put(superNode.getKey().hashCode(), node);
      }

      long startTime = System.nanoTime();
      updateNormals();
      long endTime = System.nanoTime();
      System.out.println("Exiting  updateNormals took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

      startTime = System.nanoTime();
      updatePlanarRegionSegmentation(depth);
      endTime = System.nanoTime();
      System.out.println("Exiting  updatePlanarRegionSegmentation took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));
   }

   public void updateSweepCollectionHitLocations(SweepCollection sweepCollection, double alphaUpdate)
   {
      for (int i = 0; i < sweepCollection.getNumberOfSweeps(); i++)
         updateHitLocations(sweepCollection.getSweepOrigin(i), sweepCollection.getSweep(i), alphaUpdate);
   }

   private final OcTreeKey hitLocationKey = new OcTreeKey();

   public void updateHitLocations(Point3d sensorOrigin, PointCloud pointCloud, double alphaUpdate)
   {
      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point3f point = pointCloud.getPoint(i);
         if (!isInBoundingBox(point))
            continue;
         if (coordinateToKey(point, hitLocationKey))
            updateNodeCenterRecursively(root, hitLocationKey, 0, sensorOrigin, point, alphaUpdate);
      }
   }

   private void updateNodeCenterRecursively(NormalOcTreeNode node, OcTreeKeyReadOnly key, int depth, Point3d sensorOrigin, Point3f centerUpdate, double alphaUpdate)
   {
      if (depth < treeDepth)
      {
         int childIndex = OcTreeKeyTools.computeChildIndex(key, treeDepth - 1 - depth);
         NormalOcTreeNode child;
         if (node.hasArrayForChildren() && (child = node.getChildUnsafe(childIndex)) != null)
            updateNodeCenterRecursively(child, key, depth + 1, sensorOrigin, centerUpdate, alphaUpdate);
      }
      node.updateCenter(centerUpdate, alphaUpdate);
      if (!node.isNormalSet())
      {
         tempCenterUpdate.set(centerUpdate);
         tempInitialNormalGuess.sub(sensorOrigin, tempCenterUpdate);
         tempInitialNormalGuess.normalize();
         node.setNormal(tempInitialNormalGuess);
         node.setNormalQuality(Float.POSITIVE_INFINITY);
      }
   }

   public void updateNormals()
   {
      leafIterable.setMaxDepth(treeDepth);
      boolean unknownStatus = true;
      double searchRadius = 0.05; //2.1 * getNodeSize(treeDepth);

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable)
      {
         NormalOcTreeNode node = superNode.getNode();
         OcTreeKeyReadOnly key = superNode.getKey();
         int depth = superNode.getDepth();

         if (!isNodeOccupied(node))
         {
            node.resetNormal();
            continue;
         }

//         if (node.getNormalQuality() < 0.005 && random.nextInt(5) != 0)
//         {
//            continue;
//         }

         switch (NORMAL_COMPUTATION_METHOD)
         {
         case RANSAC:
            computeNodeNormalRansac(node, key, depth, unknownStatus, searchRadius);
            break;
         case CLOSE_NEIGHBORS:
            computeNodeNormalWithSphericalNeighborhood(node, key, depth, unknownStatus, searchRadius);
            break;
         case DIRECT_NEIGHBORS:
            computeNodeNormalWithDirectNeighbors(node, key, depth, unknownStatus);
            break;
         default:
            break;
         }
      }

      if (root != null)
         updateInnerNormalsRecursive(root, 0);
   }

   private void updateInnerNormalsRecursive(NormalOcTreeNode node, int depth)
   {
      // only recurse and update for inner nodes:
      if (node.hasAtLeastOneChild())
      {
         // return early for last level:
         if (depth < treeDepth - 1)
         {
            for (int i = 0; i < 8; i++)
            {
               NormalOcTreeNode childNode;
               if ((childNode = node.getChildUnsafe(i)) != null)
                  updateInnerNormalsRecursive(childNode, depth + 1);
            }
         }
         node.resetRegionId();
         node.updateNormalChildren();
      }
   }

   private void computeNodeNormalWithDirectNeighbors(NormalOcTreeNode node, OcTreeKeyReadOnly key, int depth, boolean unknownStatus)
   {
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
   private final OcTreeKey tempKeyForNormal = new OcTreeKey();
   private final Vector3d normal = new Vector3d();
   private final Vector3d nodeCenterToNeighborCenter = new Vector3d();
   private final Point3d nodeCenter = new Point3d();
   private final Point3d neighborCenter = new Point3d();

   private void computeNodeNormalWithSphericalNeighborhood(NormalOcTreeNode node, OcTreeKeyReadOnly key, int depth, boolean unknownStatus, double searchRadius)
   {
      if (node.isCenterSet())
         node.getCenter(nodeCenter);
      else
         keyToCoordinate(key, nodeCenter, depth);

      NormalOcTreeNode currentNode;

      OcTreeKeyList cachedNeighborKeyOffsets = getCachedNeighborKeyOffsets(depth, searchRadius);
      tempNeighborKeysForNormal.clear();
      for (int i = 0; i < cachedNeighborKeyOffsets.size(); i++)
      {
         OcTreeKey currentKey = tempNeighborKeysForNormal.add();
         currentKey.set(key);
         currentKey.add(cachedNeighborKeyOffsets.unsafeGet(i));
         if (!OcTreeKeyTools.isKeyValid(currentKey, depth, treeDepth))
            tempNeighborKeysForNormal.removeLast();
      }

      normal.set(0.0, 0.0, 0.0);

      for (int i = tempNeighborKeysForNormal.size() - 1; i >= 0; i--)
      {
         OcTreeKey currentKey = tempNeighborKeysForNormal.unsafeGet(i);
         currentNode = keyToNodeMap.get(currentKey.hashCode());

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
            currentNode = keyToNodeMap.get(currentKey.hashCode());

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

   private final List<NormalOcTreeNode> tempNeighborsForNormal = new ArrayList<>();

   private void computeNodeNormalRansac(NormalOcTreeNode node, OcTreeKeyReadOnly key, int depth, boolean unknownStatus, double searchRadius)
   {
      if (!node.isCenterSet() || !node.isNormalQualitySet())
      {
         node.resetNormal();
//         computeNodeNormalWithSphericalNeighborhood(node, key, depth, unknownStatus, searchRadius);
         return;
      }

      node.getCenter(nodeCenter);

      NormalOcTreeNode currentNode;

      OcTreeKeyList cachedNeighborKeyOffsets = getCachedNeighborKeyOffsets(depth, searchRadius);

      tempNeighborsForNormal.clear();
      for (int i = 0; i < cachedNeighborKeyOffsets.size(); i++)
      {
         tempKeyForNormal.add(key, cachedNeighborKeyOffsets.unsafeGet(i));
         NormalOcTreeNode neighborNode = keyToNodeMap.get(tempKeyForNormal.hashCode());
         if (neighborNode == null)
            neighborNode = search(tempKeyForNormal);

         if (neighborNode == null)
            continue;
         if (!isNodeOccupied(neighborNode))
            continue;
         if (!neighborNode.isCenterSet())
            continue;
         tempNeighborsForNormal.add(neighborNode);
      }

      normalCandidate.set(0.0, 0.0, 0.0);

      int index = 0;
      node.getCenter(randomDraw[index++]);

      while (index < 3)
      {
         // Did not find three points, just fall back to the naive way
         if (tempNeighborsForNormal.isEmpty())
         {
//            System.err.println("Not more neighbos :/");
            node.resetNormal();
//          computeNodeNormalWithSphericalNeighborhood(node, key, depth, unknownStatus, searchRadius);
            return;
         }

         int nextInt = random.nextInt(tempNeighborsForNormal.size());
         currentNode = tempNeighborsForNormal.get(nextInt);
         currentNode.getCenter(randomDraw[index++]);
         Collections.swap(tempNeighborsForNormal, nextInt, tempNeighborsForNormal.size() - 1);
         tempNeighborsForNormal.remove(tempNeighborsForNormal.size() - 1);
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

      for (int i = 0; i < tempNeighborsForNormal.size(); i++)
      {
         currentNode = tempNeighborsForNormal.get(i);
         currentNode.getCenter(neighborCenter);
         nodeCenterToNeighborCenter.sub(nodeCenter, neighborCenter);
         normalQuality += Math.abs(normalCandidate.dot(nodeCenterToNeighborCenter));
         numberOfPoints++;
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

   public OcTreeKeyList getCachedNeighborKeyOffsets(int depth, double searchRadius)
   {
      TIntObjectHashMap<OcTreeKeyList> radiusBasedCache = neighborOffsetsCached.get(searchRadius);
      if (radiusBasedCache == null)
      {
         radiusBasedCache = new TIntObjectHashMap<>(4);
         neighborOffsetsCached.put(searchRadius, radiusBasedCache);
      }

      OcTreeKeyList cachedNeighborOffsets = radiusBasedCache.get(depth);
      if (cachedNeighborOffsets == null)
      {
         cachedNeighborOffsets = new OcTreeKeyList();
         radiusBasedCache.put(depth, cachedNeighborOffsets);
         OcTreeKeyTools.computeNeighborKeyOffsets(depth, resolution, treeDepth, searchRadius, cachedNeighborOffsets);
      }
      return cachedNeighborOffsets;
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
      double exploringRadius = 5.0 * getNodeSize(depth);
      double maxMistanceFromPlane = 0.05;
      double angleThreshold = Math.toRadians(10.0);
      double dotThreshold = Math.cos(angleThreshold);
      Random random = new Random(45561L);
      Vector3d nodeNormal = new Vector3d();

      leafIterable.setMaxDepth(depth);

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable)
      {
         NormalOcTreeNode node = superNode.getNode();

         if (!isNodeOccupied(node) || node.isPartOfRegion() || !node.isNormalSet())
            continue;
         if (node.getNormalQuality() > 0.005)
            continue;

         OcTreeKeyReadOnly nodeKey = superNode.getKey();
         int regionId = random.nextInt(Integer.MAX_VALUE);
         PlanarRegion planarRegion = new PlanarRegion(regionId);
         node.getNormal(nodeNormal);
         planarRegion.update(nodeNormal, keyToCoordinate(nodeKey, depth));
         node.setRegionId(planarRegion.getId());
         node.setHasBeenCandidateForRegion(planarRegion.getId());

         growPlanarRegionIteratively(planarRegion, nodeKey, depth, exploringRadius, maxMistanceFromPlane, dotThreshold);
      }
   }

   private final Vector3d normalCandidateToCurrentRegion = new Vector3d();
   private final Point3d centerCandidateToCurrentRegion = new Point3d();
   private final OcTreeKeyDeque keysToExplore = new OcTreeKeyDeque();
   private final ArrayDeque<NormalOcTreeNode> nodesToExplore = new ArrayDeque<>();
   private final OcTreeKey neighborKey = new OcTreeKey();

   private void growPlanarRegionIteratively(PlanarRegion planarRegion, OcTreeKeyReadOnly nodeKey, int depth, double searchRadius, double maxMistanceFromPlane,
         double dotThreshold)
   {
      keysToExplore.clear();

      OcTreeKeyList cachedNeighborKeyOffsets = getCachedNeighborKeyOffsets(depth, searchRadius);

      int planarRegionId = planarRegion.getId();
      extendSearch(nodeKey, depth, cachedNeighborKeyOffsets, planarRegionId);

      while (!keysToExplore.isEmpty())
      {
         OcTreeKey currentKey = keysToExplore.poll();
         NormalOcTreeNode currentNode = nodesToExplore.poll(); //keyToNodeMap.get(currentKey.hashCode());

         currentNode.getNormal(normalCandidateToCurrentRegion);
         currentNode.getCenter(centerCandidateToCurrentRegion);

         double dot = planarRegion.dot(normalCandidateToCurrentRegion);
         if (planarRegion.absoluteOrthogonalDistance(centerCandidateToCurrentRegion) < maxMistanceFromPlane && Math.abs(dot) > dotThreshold)
         {
            planarRegion.update(normalCandidateToCurrentRegion, centerCandidateToCurrentRegion);
            currentNode.setRegionId(planarRegionId);

            if (dot < 0.0)
               currentNode.negateNormal();

            extendSearch(currentKey, depth, cachedNeighborKeyOffsets, planarRegionId);
         }
      }
   }

   private void extendSearch(OcTreeKeyReadOnly nodeKey, int depth, OcTreeKeyList cachedNeighborKeyOffsets, int planarRegionId)
   {
      for (int i = 0; i < cachedNeighborKeyOffsets.size(); i++)
      {
         neighborKey.add(nodeKey, cachedNeighborKeyOffsets.unsafeGet(i));
         NormalOcTreeNode neighborNode = keyToNodeMap.get(neighborKey.hashCode());
         if (neighborNode == null || !isNodeOccupied(neighborNode))
            continue;
         if (neighborNode.getHasBeenCandidateForRegion() == planarRegionId || neighborNode.isPartOfRegion())
            continue;
         if (!neighborNode.isNormalSet() || !neighborNode.isCenterSet())
            continue;
         neighborNode.setHasBeenCandidateForRegion(planarRegionId);
         keysToExplore.add(neighborKey);
         nodesToExplore.add(neighborNode);
      }
   }

   @Override
   protected Class<NormalOcTreeNode> getNodeClass()
   {
      return NormalOcTreeNode.class;
   }
}
