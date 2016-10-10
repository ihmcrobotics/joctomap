package us.ihmc.octoMap.ocTree;

import static us.ihmc.robotics.geometry.GeometryTools.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.stream.Stream;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.time.StopWatch;
import org.apache.commons.math3.util.Precision;

import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.octoMap.occupancy.OccupancyParameters;
import us.ihmc.octoMap.occupancy.OccupancyParametersReadOnly;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.octoMap.pointCloud.Scan;
import us.ihmc.octoMap.pointCloud.ScanCollection;
import us.ihmc.octoMap.rules.NormalOcTreeHitUpdateRule;
import us.ihmc.octoMap.rules.NormalOcTreeMissUpdateRule;
import us.ihmc.octoMap.rules.interfaces.RayActionRule;
import us.ihmc.octoMap.tools.NormalEstimationTools;
import us.ihmc.octoMap.tools.OcTreeRayTools;
import us.ihmc.octoMap.tools.OccupancyTools;
import us.ihmc.octoMap.tools.OctoMapTools;

public class NormalOcTree extends AbstractOcTreeBase<NormalOcTreeNode>
{
   private static final boolean REPORT_TIME = true;
   private final StopWatch stopWatch = REPORT_TIME ? new StopWatch() : null;

   private static final boolean COMPUTE_NORMALS_IN_PARALLEL = true;
   private static final boolean COMPUTE_UPDATES_IN_PARALLEL = true;
   private static final boolean USE_KEY_NODE_MAP_WITH_UPDATES = true;
   private static final boolean USE_KEY_NODE_MAP_WITH_NORMALS = true;

   public static final boolean UPDATE_NODE_HIT_WITH_AVERAGE = true;

   private final String name = getClass().getSimpleName();

   // occupancy parameters of tree, stored in logodds:
   private final OccupancyParameters occupancyParameters = new OccupancyParameters();
   private final NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
   private OcTreeBoundingBoxInterface boundingBox = null;
   /** Minimum range for how long individual beams are inserted (default -1: complete beam) when inserting a ray or point cloud */
   private double minInsertRange = -1.0;
   /** Maximum range for how long individual beams are inserted (default -1: complete beam) when inserting a ray or point cloud */
   private double maxInsertRange = -1.0;

   private final NormalOcTreeHitUpdateRule hitUpdateRule = new NormalOcTreeHitUpdateRule(occupancyParameters);
   private final NormalOcTreeMissUpdateRule missUpdateRule = new NormalOcTreeMissUpdateRule(occupancyParameters);

   private double alphaCenterUpdate = 0.1;

   private final List<OcTreeKey> keyList = new ArrayList<>();
   private final List<NormalOcTreeNode> leafNodes = new ArrayList<>();
   private final HashMap<OcTreeKey, NormalOcTreeNode> keyToNodeMap = new HashMap<>();
   
   public NormalOcTree(double resolution)
   {
      super(resolution);
   }

   public NormalOcTree(NormalOcTree other)
   {
      super(other);
   }

   public void update(ScanCollection sweepCollection)
   {
      update(sweepCollection, true);
   }

   public void update(ScanCollection scanCollection, boolean updateNormals)
   {
      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      insertSweepCollection(scanCollection);

      if (REPORT_TIME)
      {
         System.out.println(name + ": ScanCollection integration took: " + OctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()) + " sec. (number of points: " + scanCollection.getNumberOfPoints() + ").");
      }

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      

      keyToNodeMap.clear();
      keyList.clear();
      leafNodes.clear();

      for (NormalOcTreeNode node : this)
      {
         OcTreeKey key = new OcTreeKey();
         node.getKey(key);
         keyToNodeMap.put(key, node);
         keyList.add(key);
         leafNodes.add(node);
      }

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      

      if (updateNormals)
      {
         if (REPORT_TIME)
         {
            stopWatch.reset();
            stopWatch.start();
         }

         updateNormals();

         if (REPORT_TIME)
         {
            System.out.println(name + ": Normal computation took: " + OctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()) + " sec.");
         }
      }
   }

   private void insertSweepCollection(ScanCollection scanCollection)
   {
      scanCollection.forEach(this::insertScan);
   }

   private final HashSet<OcTreeKey> occupiedCells = new HashSet<>();
   private final ConcurrentLinkedQueue<OcTreeKeyReadOnly> freeKeysToUpdate = new ConcurrentLinkedQueue<>();

   private final RayActionRule integrateMissActionRule = (rayOrigin, rayEnd, rayDirection, key) -> doRayActionOnFreeCell(rayOrigin, rayEnd, rayDirection, key);

   private void insertScan(Scan scan)
   {
      missUpdateRule.setUpdateLogOdds(occupancyParameters.getMissProbabilityLogOdds());
      hitUpdateRule.setUpdateLogOdds(occupancyParameters.getHitProbabilityLogOdds());
      hitUpdateRule.setAlphaHitLocationUpdate(alphaCenterUpdate);
      occupiedCells.clear();

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
            if (!occupiedCells.add(occupiedKey))
               pointCloud.removePoint(i);
         }
      }

      Stream<Point3f> stream = COMPUTE_UPDATES_IN_PARALLEL ? pointCloud.parallelStream() : pointCloud.stream();

      stream.forEach(scanPoint -> insertMissRay(sensorOrigin, scanPoint));

      if (COMPUTE_UPDATES_IN_PARALLEL)
      {
         while (!freeKeysToUpdate.isEmpty())
            updateNodeInternal(freeKeysToUpdate.poll(), missUpdateRule, missUpdateRule);
      }
   }

   private void insertMissRay(Point3d sensorOrigin, Point3f scanPoint)
   {
      Point3d point = new Point3d();
      Vector3d direction = new Vector3d();

      point.set(scanPoint);
      direction.sub(point, sensorOrigin);
      double length = direction.length();

      if (minInsertRange >= 0.0 && length < minInsertRange)
         return;

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

      OcTreeRayTools.doActionOnRayKeys(sensorOrigin, rayEnd, boundingBox, integrateMissActionRule, resolution, treeDepth);
   }

   private void doRayActionOnFreeCell(Point3d rayOrigin, Point3d rayEnd, Vector3d rayDirection, OcTreeKeyReadOnly key)
   {
      NormalOcTreeNode node = USE_KEY_NODE_MAP_WITH_UPDATES ? keyToNodeMap.get(key) : search(key);
      if (node != null && !occupiedCells.contains(key))
      {
         if (node.getNormalConsensusSize() > 10 && node.isHitLocationSet() && node.isNormalSet())
         {
            Point3d nodeHitLocation = new Point3d();
            Vector3d nodeNormal = new Vector3d();
            node.getHitLocation(nodeHitLocation);
            node.getNormal(nodeNormal);

            if (Precision.equals(Math.abs(nodeNormal.angle(rayDirection)) - Math.PI / 2.0, 0.0, Math.toRadians(30.0)) && distanceFromPointToLine(nodeHitLocation, rayOrigin, rayEnd) > 0.005)
               return;
         }
         if (COMPUTE_UPDATES_IN_PARALLEL)
            freeKeysToUpdate.offer(new OcTreeKey(key));
         else
            updateNodeInternal(key, missUpdateRule, missUpdateRule);
      }
   }

   private void updateNormals()
   {
      double searchRadius = normalEstimationParameters.getSearchRadius();
      double maxDistanceFromPlane = normalEstimationParameters.getMaxDistanceFromPlane();

      if (USE_KEY_NODE_MAP_WITH_NORMALS)
         NormalEstimationTools.getCachedNeighborKeyOffsets(searchRadius, resolution, treeDepth);

      Stream<OcTreeKey> keyStream = COMPUTE_NORMALS_IN_PARALLEL ? keyList.stream() : keyList.parallelStream();
      if (USE_KEY_NODE_MAP_WITH_NORMALS)
         keyStream.forEach(key -> NormalEstimationTools.computeNodeNormalRansac(key, keyToNodeMap, searchRadius, maxDistanceFromPlane, resolution, treeDepth));
      else
         keyStream.forEach(key -> NormalEstimationTools.computeNodeNormalRansac(root, keyToNodeMap.get(key), searchRadius, maxDistanceFromPlane));

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
}
