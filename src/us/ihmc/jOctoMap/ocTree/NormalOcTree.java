package us.ihmc.jOctoMap.ocTree;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.time.StopWatch;
import org.apache.commons.math3.util.Pair;

import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.jOctoMap.occupancy.OccupancyParameters;
import us.ihmc.jOctoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.jOctoMap.rules.NormalOcTreeHitUpdateRule;
import us.ihmc.jOctoMap.rules.NormalOcTreeMissUpdateRule;
import us.ihmc.jOctoMap.rules.interfaces.RayActionRule;
import us.ihmc.jOctoMap.tools.JOctoMapTools;
import us.ihmc.jOctoMap.tools.NormalEstimationTools;
import us.ihmc.jOctoMap.tools.OcTreeRayTools;
import us.ihmc.jOctoMap.tools.OccupancyTools;

public class NormalOcTree extends AbstractOcTreeBase<NormalOcTreeNode>
{
   private final StopWatch stopWatch = new StopWatch();

   private final String name = getClass().getSimpleName();

   // occupancy parameters of tree, stored in logodds:
   private final OccupancyParameters occupancyParameters = new OccupancyParameters();
   private final NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
   private OcTreeBoundingBoxInterface boundingBox = null;
   /** Minimum range for how long individual beams are inserted (default -1: complete beam) when inserting a ray or point cloud */
   private double minInsertRange = -1.0;
   /** Maximum range for how long individual beams are inserted (default -1: complete beam) when inserting a ray or point cloud */
   private double maxInsertRange = -1.0;
   /**
    * Specifies the maximum number of hits for the nodes.
    * This parameters changes how new hit updates are taken into account when updating nodes.
    * A lower number implies a quicker updates of new hit locations.
    */
   private long nodeMaximumNumberOfHits = Long.MAX_VALUE;

   private boolean computeNormalsInParallel = false;
   private boolean insertMissesInParallel = false;

   private boolean reportTime = false;

   private final NormalOcTreeHitUpdateRule hitUpdateRule = new NormalOcTreeHitUpdateRule(occupancyParameters);
   private final NormalOcTreeMissUpdateRule missUpdateRule = new NormalOcTreeMissUpdateRule(occupancyParameters);
   private RayMissProbabilityUpdater rayMissProbabilityUpdater = null;

   public NormalOcTree(double resolution)
   {
      super(resolution);
   }

   public NormalOcTree(NormalOcTree other)
   {
      super(other);
   }

   public void update(ScanCollection scanCollection)
   {
      insertScanCollection(scanCollection, true);
      updateNormals();
   }

   public void insertNormalOcTree(Point3d sensorOrigin, NormalOcTree otherOcTree)
   {
      insertNormalOcTree(sensorOrigin, otherOcTree, null);
   }

   public void insertNormalOcTree(Point3d sensorOrigin, NormalOcTree otherOcTree, Matrix4d otherOcTreeTransform)
   {
      if (reportTime)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      missUpdateRule.setUpdateLogOdds(occupancyParameters.getMissProbabilityLogOdds());
      hitUpdateRule.setUpdateLogOdds(occupancyParameters.getHitProbabilityLogOdds());
      hitUpdateRule.setMaximumNumberOfHits(nodeMaximumNumberOfHits);
      HashSet<OcTreeKey> occupiedCells = new HashSet<>();

      Vector3d direction = new Vector3d();
      Point3d point = new Point3d();
      PointCloud pointCloudForIntegratingMiss = new PointCloud();

      for (NormalOcTreeNode otherNode : otherOcTree)
      {
         otherNode.getHitLocation(point);
         if (otherOcTreeTransform != null)
            otherOcTreeTransform.transform(point);
         
         direction.sub(point, sensorOrigin);
         double length = direction.length();

         if ((maxInsertRange < 0.0 || length <= maxInsertRange) && (minInsertRange < 0.0 || length >= minInsertRange) && isInBoundingBox(point))
         {
            OcTreeKey occupiedKey = coordinateToKey(point);
            if (occupiedKey == null)
               continue;

            hitUpdateRule.setHitLocation(sensorOrigin, point);
            hitUpdateRule.setHitUpdateWeight(otherNode.getNumberOfHits());

            updateNodeInternal(point, hitUpdateRule, null);

            if (occupiedCells.add(occupiedKey))
               pointCloudForIntegratingMiss.add(point);
         }
         else
         {
            pointCloudForIntegratingMiss.add(point);
         }
      }

      insertMissRays(sensorOrigin, pointCloudForIntegratingMiss, occupiedCells);

      if (reportTime)
      {
         System.out.println(name + ": Insert OcTree took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()) + " sec.");
      }
   }

   public void insertScanCollection(ScanCollection scanCollection)
   {
      insertScanCollection(scanCollection, true);
   }

   public void insertScanCollection(ScanCollection scanCollection, boolean insertMiss)
   {
      if (reportTime)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      scanCollection.forEach(scan -> insertScan(scan, insertMiss));

      if (reportTime)
      {
         System.out.println(name + ": ScanCollection integration took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()) + " sec. (number of points: " + scanCollection.getNumberOfPoints() + ").");
      }
   }

   public void updateNormals()
   {
      if (reportTime)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      List<NormalOcTreeNode> leafNodes = new ArrayList<>();
      this.forEach(leafNodes::add);
      Stream<NormalOcTreeNode> nodeStream = computeNormalsInParallel ? leafNodes.parallelStream() : leafNodes.stream();
      nodeStream.forEach(node -> NormalEstimationTools.computeNodeNormalRansac(root, node, normalEstimationParameters));

      if (root != null)
         updateInnerNormalsRecursive(root, 0);

      if (reportTime)
      {
         System.out.println(name + ": Normal computation took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()) + " sec.");
      }
   }

   public void clearNormals()
   {
      List<NormalOcTreeNode> leafNodes = new ArrayList<>();
      this.forEach(leafNodes::add);
      leafNodes.stream().forEach(NormalOcTreeNode::resetNormal);
   }

   private void insertScan(Scan scan, boolean insertMiss)
   {
      missUpdateRule.setUpdateLogOdds(occupancyParameters.getMissProbabilityLogOdds());
      hitUpdateRule.setUpdateLogOdds(occupancyParameters.getHitProbabilityLogOdds());
      hitUpdateRule.setMaximumNumberOfHits(nodeMaximumNumberOfHits);

      HashSet<OcTreeKey> occupiedCells = new HashSet<>();

      Vector3d direction = new Vector3d();
      Point3d point = new Point3d();
      Point3d sensorOrigin = scan.getSensorOrigin();
      PointCloud pointCloud = scan.getPointCloud();

      for (int i = pointCloud.getNumberOfPoints() - 1; i >= 0; i--)
      {
         point.set(pointCloud.getPoint(i));
         direction.sub(point, sensorOrigin);
         double length = direction.length();

         if ((maxInsertRange < 0.0 || length <= maxInsertRange) && (minInsertRange < 0.0 || length >= minInsertRange) && isInBoundingBox(point))
         {
            OcTreeKey occupiedKey = coordinateToKey(point);
            if (occupiedKey == null)
               continue;
            hitUpdateRule.setHitLocation(sensorOrigin, point);
            updateNodeInternal(occupiedKey, hitUpdateRule, null);
            // Add the key to the occupied set.
            // if it was already present, remove the point from the scan to speed up integration of miss.
            if (!occupiedCells.add(occupiedKey) && insertMiss)
               pointCloud.removePoint(i);
         }
      }

      if (insertMiss)
      {
         insertMissRays(sensorOrigin, pointCloud, occupiedCells);
      }
   }

   private void insertMissRays(Point3d sensorOrigin, PointCloud pointCloud, HashSet<OcTreeKey> occupiedCells)
   {
      Map<OcTreeKey, NormalOcTreeNode> keyToNodeMap = new HashMap<>();
      this.forEach(node -> keyToNodeMap.put(node.getKeyCopy(), node));
      Stream<Point3f> pointCloudStream = insertMissesInParallel ? pointCloud.parallelStream() : pointCloud.stream();

      List<List<Pair<OcTreeKey, Float>>> keysAndMissUpdates;
      keysAndMissUpdates = pointCloudStream.map(scanPoint -> insertMissRay(sensorOrigin, scanPoint, occupiedCells, keyToNodeMap))
                                           .filter(list -> list != null)
                                           .collect(Collectors.toList());

      for (List<Pair<OcTreeKey, Float>> list : keysAndMissUpdates)
      {
         // This has to be sequential as the octree is modified
         list.stream()
               .forEach(keyAndMissUpdate -> {
                  missUpdateRule.setUpdateLogOdds(keyAndMissUpdate.getValue());
                  updateNodeInternal(keyAndMissUpdate.getKey(), missUpdateRule, missUpdateRule);
               });
      }
   }

   private List<Pair<OcTreeKey, Float>> insertMissRay(Point3d sensorOrigin, Point3f scanPoint, Set<OcTreeKey> occupiedCells, Map<OcTreeKey, NormalOcTreeNode> keyToNodeMap)
   {
      return insertMissRay(sensorOrigin, new Point3d(scanPoint), occupiedCells, keyToNodeMap);
   }

   private List<Pair<OcTreeKey, Float>> insertMissRay(Point3d sensorOrigin, Point3d scanPoint, Set<OcTreeKey> occupiedCells, Map<OcTreeKey, NormalOcTreeNode> keyToNodeMap)
   {
      Vector3d direction = new Vector3d();
      direction.sub(scanPoint, sensorOrigin);
      double length = direction.length();

      if (minInsertRange >= 0.0 && length < minInsertRange)
         return null;

      Point3d rayEnd;
      if (maxInsertRange < 0.0 || length <= maxInsertRange)
      { // is not maxrange meas, free cells
         rayEnd = scanPoint;
      }
      else
      { // user set a maxrange and length is above
         rayEnd = new Point3d();
         rayEnd.scaleAdd(maxInsertRange / length, direction, sensorOrigin);
      } // end if maxrange

      List<Pair<OcTreeKey, Float>> keysAndMissUpdates = new ArrayList<>();

      RayActionRule integrateMissActionRule = new RayActionRule()
      {
         @Override
         public void doAction(Point3d rayOrigin, Point3d rayEnd, Vector3d rayDirection, OcTreeKeyReadOnly key)
         {
            Pair<OcTreeKey, Float> keyAndMissUpdate = doRayActionOnFreeCell(rayOrigin, rayEnd, rayDirection, key, occupiedCells, keyToNodeMap);
            if (keyAndMissUpdate != null)
               keysAndMissUpdates.add(keyAndMissUpdate);
         }
      };
      OcTreeRayTools.doActionOnRayKeys(sensorOrigin, rayEnd, boundingBox, integrateMissActionRule, resolution, treeDepth);

      return keysAndMissUpdates;
   }

   private Pair<OcTreeKey, Float> doRayActionOnFreeCell(Point3d rayOrigin, Point3d rayEnd, Vector3d rayDirection, OcTreeKeyReadOnly key, Set<OcTreeKey> occupiedCells, Map<OcTreeKey, NormalOcTreeNode> keyToNodeMap)
   {
      if (occupiedCells.contains(key))
         return null;

      float updateLogOdds = occupancyParameters.getMissProbabilityLogOdds();

      if (rayMissProbabilityUpdater != null)
      {
         NormalOcTreeNode node = keyToNodeMap.get(key);

         if (node == null)
            return null;

         double updateProbability = rayMissProbabilityUpdater.computeRayMissProbability(rayOrigin, rayEnd, rayDirection, node, occupancyParameters);
         updateLogOdds = JOctoMapTools.logodds(updateProbability);
      }

      return new Pair<>(new OcTreeKey(key), updateLogOdds);
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
               NormalOcTreeNode childNode = node.getChild(i);
               if (childNode != null)
                  updateInnerNormalsRecursive(childNode, depth + 1);
            }
         }
         node.updateNormalChildren();
      }
   }

   @Override
   public Iterator<NormalOcTreeNode> iterator()
   {
      return OcTreeIteratorFactory.createLeafBoundingBoxIteratable(root, boundingBox).iterator();
   }

   /**
    * When set to true, the computation time for updating the octree is printed out in the console.
    * @param enable whether to report computation time or not.
    */
   public void enableReportTime(boolean enable)
   {
      reportTime = enable;
   }

   /**
    * When updated with a hit location, each node is computing the average of where it has been hit over time.
    * The average is computed by keeping track of how many times a node has been hit.
    * By limiting the number of hits, the user has access on the weight that a new hit represents.
    * A low value will correspond to maintaining quick updates over time, while a high value will reduce the update over time.
    * @param maximumNumberOfHits
    */
   public void setNodeMaximumNumberOfHits(long maximumNumberOfHits)
   {
      if (maximumNumberOfHits <= 0)
         throw new RuntimeException("Unexpected value for maximumNumberOfHits. Expected a value in [1, Long.MAX_VALUE], but was: " + maximumNumberOfHits);
      this.nodeMaximumNumberOfHits = maximumNumberOfHits;
   }

   /**
    * Changes how the normals for each occupied is evaluated, either in parallel or sequential.
    * The parallel computation offers quicker updates at the cost of an increased CPU usage.
    * @param enable whether to compute the normals in parallel or sequential.
    */
   public void enableParallelComputationForNormals(boolean enable)
   {
      computeNormalsInParallel = enable;
   }

   /**
    * When updating the occupancy of the octree, most of the time is spent updating the misses.
    * Even if expensive, miss updates are useful to make the octree to a changing environment.
    * The parallel computation offers quicker updates at the cost of an increased CPU usage.
    * @param enable whether to insert the misses in parallel or sequential.
    */
   public void enableParallelInsertionOfMisses(boolean enable)
   {
      insertMissesInParallel = enable;
   }

   /**
    * Set a custom updater to compute the probability of a miss when a node is traversed by a ray.
    * @param rayMissProbabilityUpdater the custom update to use.
    */
   public void setCustomRayMissProbabilityUpdater(RayMissProbabilityUpdater rayMissProbabilityUpdater)
   {
      this.rayMissProbabilityUpdater = rayMissProbabilityUpdater;
   }

   public void disableCustomRayMissProbabilityUpdater()
   {
      this.rayMissProbabilityUpdater = null;
   }

   public void disableBoundingBox()
   {
      boundingBox = null;
   }

   /**
    * Bounding box to use for the next updates on this OcTree.
    * If null, no limit will be applied.
    * @param boundingBox
    */
   public void setBoundingBox(OcTreeBoundingBoxInterface boundingBox)
   {
      this.boundingBox = boundingBox;
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

   /**
    * Provides a flexible API for computing a custom update for nodes traversed by rays.
    * Useful to reduce "self-destruction" of the octree when scanning surfaces at a shallow angle.
    * The normal contained in each node combined with the ray properties can be used to refine the probability of a miss.
    */
   public interface RayMissProbabilityUpdater
   {
      /**
       * Compute a custom miss probability update when a ray goes through a node.
       * The default implementation returns the miss probability from the occupancy parameters.
       * 
       * @param rayOrigin the origin of the ray.
       * @param rayEnd the hit location of the ray.
       * @param rayDirection the direction of the ray.
       * @param node the current node the ray is going through.
       * @param parameters the current occupancy parameters used by this octree.
       * @return the custom miss probability update.
       */
      public default double computeRayMissProbability(Point3d rayOrigin, Point3d rayEnd, Vector3d rayDirection, NormalOcTreeNode node, OccupancyParameters parameters)
      {
         return parameters.getMissProbability();
      }
   }
}
