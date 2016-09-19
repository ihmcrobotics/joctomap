package us.ihmc.octoMap.ocTree.implementations;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import gnu.trove.map.hash.TDoubleObjectHashMap;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.iterators.LeafBoundingBoxIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyDeque;
import us.ihmc.octoMap.key.OcTreeKeyList;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.octoMap.ocTree.baseImplementation.OcTreeRayHelper;
import us.ihmc.octoMap.ocTree.rules.ActionRule;
import us.ihmc.octoMap.ocTree.rules.NormalOcTreeHitUpdateRule;
import us.ihmc.octoMap.ocTree.rules.NormalOcTreeMissUpdateRule;
import us.ihmc.octoMap.occupancy.OccupancyParameters;
import us.ihmc.octoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.octoMap.occupancy.OccupancyTools;
import us.ihmc.octoMap.planarRegions.PlanarRegion;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.octoMap.pointCloud.SweepCollection;
import us.ihmc.octoMap.tools.OcTreeKeyTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.time.TimeTools;

public class NormalOcTree extends AbstractOcTreeBase<NormalOcTreeNode>
{
   public static final boolean UPDATE_NODE_HIT_WITH_AVERAGE = true;

   // occupancy parameters of tree, stored in logodds:
   private final OccupancyParameters occupancyParameters = new OccupancyParameters();
   private final NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
   private final PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
   private OcTreeBoundingBoxInterface boundingBox = null;
   /** Minimum range for how long individual beams are inserted (default -1: complete beam) when inserting a ray or point cloud */
   private double minInsertRange = -1.0;
   /** Maximum range for how long individual beams are inserted (default -1: complete beam) when inserting a ray or point cloud */
   private double maxInsertRange = -1.0;

   private final NormalOcTreeHitUpdateRule hitUpdateRule = new NormalOcTreeHitUpdateRule(occupancyParameters);
   private final NormalOcTreeMissUpdateRule missUpdateRule = new NormalOcTreeMissUpdateRule(occupancyParameters);

   private final OcTreeRayHelper<NormalOcTreeNode> rayHelper = new OcTreeRayHelper<>();

   private enum NormalComputationMethod
   {
      DIRECT_NEIGHBORS, CLOSE_NEIGHBORS, RANSAC
   };

   private static final NormalComputationMethod NORMAL_COMPUTATION_METHOD = NormalComputationMethod.RANSAC;

   private final TDoubleObjectHashMap<OcTreeKeyList> neighborOffsetsCached = new TDoubleObjectHashMap<>(4);
   private final LeafBoundingBoxIterable<NormalOcTreeNode> leafIterable;

   private double alphaCenterUpdate = 0.1;

   public NormalOcTree(double resolution)
   {
      super(resolution);
      leafIterable = new LeafBoundingBoxIterable<>(this, treeDepth, true);
   }

   public void update(SweepCollection sweepCollection)
   {
      System.out.println("Entering updateNodeFromSweepCollection sweep size: " + sweepCollection.getNumberOfSweeps());
      for (int i = 0; i < sweepCollection.getNumberOfSweeps(); i++)
         System.out.println("Point cloud size: " + sweepCollection.getSweep(i).size());
      long startTime = System.nanoTime();

      insertSweepCollection(sweepCollection);

      long endTime = System.nanoTime();
      System.out.println("Exiting  updateNodeFromSweepCollection took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      

      keyToNodeMap.clear();
      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable)
      {
         NormalOcTreeNode node = superNode.getNode();
         node.resetRegionId();
         node.resetHasBeenCandidateForRegion();
         if(!isNodeOccupied(node))
            System.err.println("HAAAAAAAAAAAAAAAA");
         keyToNodeMap.put(new OcTreeKey(superNode.getKey()), node);
      }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      

      startTime = System.nanoTime();
      updateNormals();
      endTime = System.nanoTime();
      System.out.println("Exiting  updateNormals took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      

      startTime = System.nanoTime();
      updatePlanarRegionSegmentation();
      endTime = System.nanoTime();
      System.out.println("Exiting  updatePlanarRegionSegmentation took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));
   }

   private void insertSweepCollection(SweepCollection sweepCollection)
   {

      for (int i = 0; i < sweepCollection.getNumberOfSweeps(); i++)
      {
         Point3d sensorOrigin = sweepCollection.getSweepOrigin(i);
         PointCloud scan = sweepCollection.getSweep(i);

         insertPointCloud(scan, sensorOrigin);
      }
   }

   private void integrateHitsOnly(PointCloud scan, Point3d sensorOrigin)
   {
      Point3d scanPoint = new Point3d();
      double minRangeSquared = minInsertRange < 0.0 ? 0.0 : minInsertRange * minInsertRange;
      double maxRangeSquared = maxInsertRange < 0.0 ? Double.POSITIVE_INFINITY : maxInsertRange * maxInsertRange;

      hitUpdateRule.setUpdateLogOdds(occupancyParameters.getHitProbabilityLogOdds());
      hitUpdateRule.setAlphaHitLocationUpdate(alphaCenterUpdate);

      for (int i = 0; i < scan.size(); i++)
      {
         scanPoint.set(scan.getPoint(i));

         if (!isInBoundingBox(scanPoint))
            continue;

         double distanceSquared = scanPoint.distanceSquared(sensorOrigin);
         if (distanceSquared < maxRangeSquared && distanceSquared > minRangeSquared)
         {
            hitUpdateRule.setHitLocation(sensorOrigin, scanPoint);
            updateNodeInternal(scanPoint, hitUpdateRule, null);
         }
      }
   }

   private final HashSet<OcTreeKey> occupiedCells = new HashSet<>();
   private final ActionRule integrateMissActionRule = new ActionRule()
   {
      @Override
      public void doAction(OcTreeKeyReadOnly key)
      {
         if (keyToNodeMap.containsKey(key) && isInBoundingBox(key) && !occupiedCells.contains(key))
            updateNodeInternal(key, missUpdateRule, missUpdateRule);
      }
   };

   private void insertPointCloud(PointCloud scan, Point3d sensorOrigin)
   {
      missUpdateRule.setUpdateLogOdds(occupancyParameters.getMissProbabilityLogOdds());
      hitUpdateRule.setUpdateLogOdds(occupancyParameters.getHitProbabilityLogOdds());
      hitUpdateRule.setAlphaHitLocationUpdate(alphaCenterUpdate);
      occupiedCells.clear();

      Vector3d direction = new Vector3d();
      Point3d point = new Point3d();

      for (int i = 0; i < scan.size(); ++i)
      {
         point.set(scan.getPoint(i));
         direction.sub(point, sensorOrigin);
         double length = direction.length();

         if (isInBoundingBox(point) && (maxInsertRange < 0.0 || length <= maxInsertRange) && (minInsertRange < 0.0 || length >= minInsertRange))
         {
            OcTreeKey occupiedKey = coordinateToKey(point);
            occupiedCells.add(occupiedKey);
            hitUpdateRule.setHitLocation(sensorOrigin, point);
            updateNodeInternal(occupiedKey, hitUpdateRule, null);
         }
      }

      for (int i = 0; i < scan.size(); ++i)
      {
         point.set(scan.getPoint(i));
         direction.sub(point, sensorOrigin);
         double length = direction.length();

         if (minInsertRange >= 0.0 && length < minInsertRange)
            continue;

         Point3d rayEnd;
         if (maxInsertRange < 0.0 || length <= maxInsertRange)
         { // is not maxrange meas, free cells
            rayEnd = point;
         }
         else
         { // user set a maxrange and length is above
            rayEnd = new Point3d();
            rayEnd.scaleAdd(maxInsertRange / length, direction, sensorOrigin);
         } // end if maxrange

         rayHelper.doActionOnRayKeys(sensorOrigin, rayEnd, boundingBox, integrateMissActionRule, resolution, treeDepth);
      } // end for all points, end of parallel OMP loop
   }

   private final HashMap<OcTreeKey, NormalOcTreeNode> keyToNodeMap = new HashMap<>();

   private void updateNormals()
   {
      leafIterable.setMaxDepth(treeDepth);
      boolean unknownStatus = true;
      double searchRadius = normalEstimationParameters.getSearchRadius();
      double maxDistanceFromPlane = normalEstimationParameters.getMaxDistanceFromPlane();

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable)
      {
         NormalOcTreeNode node = superNode.getNode();
         OcTreeKeyReadOnly key = superNode.getKey();

         if (!isNodeOccupied(node))
         {
            node.resetNormal();
            continue;
         }

         if (node.getNormalQuality() < 0.005)
         {
            if (random.nextDouble() >= 0.10)
               continue;
//            else if (random.nextDouble() <= 0.01)
//               node.resetNormal();
         }

         switch (NORMAL_COMPUTATION_METHOD)
         {
         case RANSAC:
            computeNodeNormalRansac(node, key, searchRadius, maxDistanceFromPlane);
            break;
         case CLOSE_NEIGHBORS:
            computeNodeNormalWithSphericalNeighborhood(node, key, unknownStatus, searchRadius);
            break;
         case DIRECT_NEIGHBORS:
            computeNodeNormalWithDirectNeighbors(node, key, unknownStatus);
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

   private void computeNodeNormalWithDirectNeighbors(NormalOcTreeNode node, OcTreeKeyReadOnly key, boolean unknownStatus)
   {
      OcTreeKey currentKey = new OcTreeKey();
      NormalOcTreeNode currentNode;

      int keyInterval = OcTreeKeyTools.computeKeyIntervalAtDepth(treeDepth, treeDepth);

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
               currentNode = search(currentKey);

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

   private void computeNodeNormalWithSphericalNeighborhood(NormalOcTreeNode node, OcTreeKeyReadOnly key, boolean unknownStatus, double searchRadius)
   {
      if (node.isCenterSet())
         node.getCenter(nodeCenter);
      else
         keyToCoordinate(key, nodeCenter);

      NormalOcTreeNode currentNode;

      OcTreeKeyList cachedNeighborKeyOffsets = getCachedNeighborKeyOffsets(searchRadius);
      tempNeighborKeysForNormal.clear();
      for (int i = 0; i < cachedNeighborKeyOffsets.size(); i++)
      {
         OcTreeKey currentKey = tempNeighborKeysForNormal.add();
         currentKey.set(key);
         currentKey.add(cachedNeighborKeyOffsets.unsafeGet(i));
      }

      normal.set(0.0, 0.0, 0.0);

      for (int i = tempNeighborKeysForNormal.size() - 1; i >= 0; i--)
      {
         OcTreeKey currentKey = tempNeighborKeysForNormal.unsafeGet(i);
         currentNode = keyToNodeMap.get(currentKey);

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
            currentNode = keyToNodeMap.get(currentKey);

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

   private final RecyclingArrayList<Point3d> tempNeighborCenters = new RecyclingArrayList<>(Point3d.class);

   private void computeNodeNormalRansac(NormalOcTreeNode node, OcTreeKeyReadOnly key, double searchRadius, double maxDistanceFromPlane)
   {
      if (!node.isCenterSet() || !node.isNormalQualitySet())
      {
         node.resetNormal();
         return;
      }

      node.getNormal(normal);
      node.getCenter(nodeCenter);
      double nodeNormalQuality = 0.0; // Need to be recomputed as the neighbors may have changed
      int nodeNumberOfPoints = 0;

      OcTreeKeyList cachedNeighborKeyOffsets = getCachedNeighborKeyOffsets(searchRadius);

      tempNeighborCenters.clear();
      for (int i = 0; i < cachedNeighborKeyOffsets.size(); i++)
      {
         tempKeyForNormal.add(key, cachedNeighborKeyOffsets.unsafeGet(i));
         NormalOcTreeNode neighborNode = keyToNodeMap.get(tempKeyForNormal);

         if (neighborNode == null)
            continue;
         if (!isNodeOccupied(neighborNode))
            continue;
         if (!neighborNode.isCenterSet())
            continue;
         Point3d neighborCenter = tempNeighborCenters.add();
         neighborNode.getCenter(neighborCenter);

         nodeCenterToNeighborCenter.sub(nodeCenter, neighborCenter);
         double distanceFromPlane = Math.abs(normal.dot(nodeCenterToNeighborCenter));
         if (distanceFromPlane <= maxDistanceFromPlane)
         {
            nodeNormalQuality += distanceFromPlane;
            nodeNumberOfPoints++;
         }
      }

      if (nodeNumberOfPoints == 0)
         nodeNormalQuality = Double.POSITIVE_INFINITY;
      else
         nodeNormalQuality /= nodeNumberOfPoints;

      normalCandidate.set(0.0, 0.0, 0.0);

      int index = 0;
      node.getCenter(randomDraw[index++]);

      while (index < 3)
      {
         // Did not find three points, just fall back to the naive way
         if (tempNeighborCenters.isEmpty())
         {
            node.resetNormal();
            return;
         }

         int nextInt = random.nextInt(tempNeighborCenters.size());
         randomDraw[index++].set(tempNeighborCenters.get(nextInt));
         tempNeighborCenters.fastRemove(nextInt);
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

      float candidateNormalQuality = 0.0f;
      int candidateNumberOfPoints = 3; // The three points picked randomly are on the plane

      for (int i = 0; i < tempNeighborCenters.size(); i++)
      {
         nodeCenterToNeighborCenter.sub(nodeCenter, tempNeighborCenters.get(i));
         double distanceFromPlane = Math.abs(normalCandidate.dot(nodeCenterToNeighborCenter));
         if (distanceFromPlane < maxDistanceFromPlane)
         {
            candidateNormalQuality += distanceFromPlane;
            candidateNumberOfPoints++;
         }
      }

      candidateNormalQuality /= candidateNumberOfPoints;

      if (candidateNumberOfPoints >= nodeNumberOfPoints && candidateNormalQuality < nodeNormalQuality)
      {
         if (normal.dot(normalCandidate) < 0.0)
            normalCandidate.negate();
         node.setNormal(normalCandidate);
         node.setNormalQuality(candidateNormalQuality);
      }
   }

   private OcTreeKeyList getCachedNeighborKeyOffsets(double searchRadius)
   {
      OcTreeKeyList cachedNeighborOffsets = neighborOffsetsCached.get(searchRadius);
      if (cachedNeighborOffsets == null)
      {
         cachedNeighborOffsets = new OcTreeKeyList();
         neighborOffsetsCached.put(searchRadius, cachedNeighborOffsets);
         OcTreeKeyTools.computeNeighborKeyOffsets(treeDepth, resolution, treeDepth, searchRadius, cachedNeighborOffsets);
      }
      return cachedNeighborOffsets;
   }

   private void updatePlanarRegionSegmentation()
   {
      double dotThreshold = Math.cos(planarRegionSegmentationParameters.getMaxAngleFromPlane());
      float minNormalQuality = (float) planarRegionSegmentationParameters.getMinNormalQuality();
      double searchRadius = planarRegionSegmentationParameters.getSearchRadius();
      double maxDistanceFromPlane = planarRegionSegmentationParameters.getMaxDistanceFromPlane();

      Random random = new Random(45561L);
      Vector3d nodeNormal = new Vector3d();

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable)
      {
         NormalOcTreeNode node = superNode.getNode();

         if (!isNodeOccupied(node) || node.isPartOfRegion() || !node.isNormalSet())
            continue;
         if (node.getNormalQuality() > minNormalQuality)
            continue;

         OcTreeKeyReadOnly nodeKey = superNode.getKey();
         int regionId = random.nextInt(Integer.MAX_VALUE);
         PlanarRegion planarRegion = new PlanarRegion(regionId);
         node.getNormal(nodeNormal);
         planarRegion.update(nodeNormal, keyToCoordinate(nodeKey));
         node.setRegionId(planarRegion.getId());
         node.setHasBeenCandidateForRegion(planarRegion.getId());

         growPlanarRegionIteratively(planarRegion, nodeKey, searchRadius, maxDistanceFromPlane, dotThreshold);
      }

      if (root != null)
         updateInnerRegionIdsRecursive(root, 0);
   }

   private final Vector3d normalCandidateToCurrentRegion = new Vector3d();
   private final Point3d centerCandidateToCurrentRegion = new Point3d();
   private final OcTreeKeyDeque keysToExplore = new OcTreeKeyDeque();
   private final ArrayDeque<NormalOcTreeNode> nodesToExplore = new ArrayDeque<>();
   private final OcTreeKey neighborKey = new OcTreeKey();

   private void growPlanarRegionIteratively(PlanarRegion planarRegion, OcTreeKeyReadOnly nodeKey, double searchRadius, double maxMistanceFromPlane,
         double dotThreshold)
   {
      keysToExplore.clear();

      OcTreeKeyList cachedNeighborKeyOffsets = getCachedNeighborKeyOffsets(searchRadius);

      int planarRegionId = planarRegion.getId();
      extendSearch(nodeKey, cachedNeighborKeyOffsets, planarRegionId);

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

            extendSearch(currentKey, cachedNeighborKeyOffsets, planarRegionId);
         }
      }
   }

   private void extendSearch(OcTreeKeyReadOnly nodeKey, OcTreeKeyList cachedNeighborKeyOffsets, int planarRegionId)
   {
      for (int i = 0; i < cachedNeighborKeyOffsets.size(); i++)
      {
         neighborKey.add(nodeKey, cachedNeighborKeyOffsets.unsafeGet(i));
         NormalOcTreeNode neighborNode = keyToNodeMap.get(neighborKey);
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

   private void updateInnerRegionIdsRecursive(NormalOcTreeNode node, int depth)
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
                  updateInnerRegionIdsRecursive(childNode, depth + 1);
            }
         }
         node.resetRegionId();
         node.updateRegionIdChildren();
      }
   }

   public void disableBoundingBox()
   {
      boundingBox = null;
      leafIterable.setBoundingBox(null);
   }

   /**
    * Bounding box to use for the next updates on this OcTree.
    * If null, no limit will be applied.
    * @param boundingBox
    */
   public void setBoundingBox(OcTreeBoundingBoxInterface boundingBox)
   {
      this.boundingBox = boundingBox;
      leafIterable.setBoundingBox(boundingBox);
   }

   public OcTreeBoundingBoxInterface getBoundingBox()
   {
      return boundingBox;
   }

   /**
    * @return true if point is in the currently set bounding box or if there is no bounding box.
    */
   public boolean isInBoundingBox(Point3d candidate)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(candidate);
   }

   /**
    * @return true if point is in the currently set bounding box or if there is no bounding box.
    */
   public boolean isInBoundingBox(Point3f candidate)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(candidate);
   }

   /**
    * @return true if key is in the currently set bounding box or if there is no bounding box.
    */
   public boolean isInBoundingBox(OcTreeKeyReadOnly candidate)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(candidate);
   }

   public boolean isNodeOccupied(NormalOcTreeNode node)
   {
      return OccupancyTools.isNodeOccupied(occupancyParameters, node);
   }

   public void setOccupancyParameters(OccupancyParameters parameters)
   {
      this.occupancyParameters.set(parameters);
   }

   public OccupancyParametersReadOnly getOccupancyParameters()
   {
      return occupancyParameters;
   }

   public void setNormalEstimationParameters(NormalEstimationParameters parameters)
   {
      this.normalEstimationParameters.set(parameters);
   }

   public NormalEstimationParameters getNormalEstimationParameters()
   {
      return normalEstimationParameters;
   }

   public void setPlanarRegionSegmentationParameters(PlanarRegionSegmentationParameters parameters)
   {
      this.planarRegionSegmentationParameters.set(parameters);
   }

   public PlanarRegionSegmentationParameters getPlanarRegionSegmentationParameters()
   {
      return planarRegionSegmentationParameters;
   }

   /** Minimum range for how long individual beams are inserted (default -1: complete beam) when inserting a ray or point cloud */
   public void setMinimumInsertRange(double minRange)
   {
      minInsertRange = minRange;
   }

   /** Maximum range for how long individual beams are inserted (default -1: complete beam) when inserting a ray or point cloud */
   public void setMaximumInsertRange(double maxRange)
   {
      maxInsertRange = maxRange;
   }

   /** Minimum and maximum range for how long individual beams are inserted (default -1: complete beam) when inserting a ray or point cloud */
   public void setBoundsInsertRange(double minRange, double maxRange)
   {
      setMinimumInsertRange(minRange);
      setMaximumInsertRange(maxRange);
   }

   /** Remove the limitation in minimum range when inserting a ray or point cloud. */
   public void removeMinimumInsertRange()
   {
      minInsertRange = -1.0;
   }

   /** Remove the limitation in maximum range when inserting a ray or point cloud. */
   public void removeMaximumInsertRange()
   {
      maxInsertRange = -1.0;
   }

   /** Remove the limitation in minimum and maximum range when inserting a ray or point cloud. */
   public void removeBoundsInsertRange()
   {
      removeMinimumInsertRange();
      removeMaximumInsertRange();
   }

   @Override
   protected Class<NormalOcTreeNode> getNodeClass()
   {
      return NormalOcTreeNode.class;
   }
}
