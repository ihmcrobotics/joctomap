package us.ihmc.jOctoMap.ocTree;

import static us.ihmc.jOctoMap.tools.JOctoMapGeometryTools.*;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.time.StopWatch;
import org.apache.commons.math3.util.Precision;

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
   private static final boolean REPORT_TIME = true;
   private final StopWatch stopWatch = REPORT_TIME ? new StopWatch() : null;

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
      insertScanCollection(scanCollection);
      updateNormals();
   }

   public void insertScanCollection(ScanCollection scanCollection)
   {
      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      scanCollection.forEach(this::insertScan);

      if (REPORT_TIME)
      {
         System.out.println(name + ": ScanCollection integration took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()) + " sec. (number of points: " + scanCollection.getNumberOfPoints() + ").");
      }
   }

   public void updateNormals()
   {

      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      List<NormalOcTreeNode> leafNodes = new ArrayList<>();
      this.forEach(leafNodes::add);
      leafNodes.parallelStream().forEach(node -> NormalEstimationTools.computeNodeNormalRansac(root, node, normalEstimationParameters));

      if (root != null)
         updateInnerNormalsRecursive(root, 0);

      if (REPORT_TIME)
      {
         System.out.println(name + ": Normal computation took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()) + " sec.");
      }
   }

   public void clearNormals()
   {
      List<NormalOcTreeNode> leafNodes = new ArrayList<>();
      this.forEach(leafNodes::add);
      leafNodes.parallelStream().forEach(NormalOcTreeNode::resetNormal);
   }

   private final HashSet<OcTreeKey> occupiedCells = new HashSet<>();
   private final ConcurrentLinkedQueue<OcTreeKeyReadOnly> freeKeysToUpdate = new ConcurrentLinkedQueue<>();

   private final RayActionRule integrateMissActionRule = (rayOrigin, rayEnd, rayDirection, key) -> doRayActionOnFreeCell(rayOrigin, rayEnd, rayDirection, key);

   private void insertScan(Scan scan)
   {
      missUpdateRule.setUpdateLogOdds(occupancyParameters.getMissProbabilityLogOdds());
      hitUpdateRule.setUpdateLogOdds(occupancyParameters.getHitProbabilityLogOdds());
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

      pointCloud.parallelStream().forEach(scanPoint -> insertMissRay(sensorOrigin, scanPoint));

      while (!freeKeysToUpdate.isEmpty())
         updateNodeInternal(freeKeysToUpdate.poll(), missUpdateRule, missUpdateRule);
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
      NormalOcTreeNode node = search(key);
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
         freeKeysToUpdate.offer(new OcTreeKey(key));
      }
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
