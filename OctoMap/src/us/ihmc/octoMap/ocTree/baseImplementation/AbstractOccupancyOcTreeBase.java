package us.ihmc.octoMap.ocTree.baseImplementation;

import static us.ihmc.octoMap.MarchingCubesTables.edgeTable;
import static us.ihmc.octoMap.MarchingCubesTables.triTable;
import static us.ihmc.octoMap.MarchingCubesTables.vertexList;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.KeyBoolMap;
import us.ihmc.octoMap.key.KeyRay;
import us.ihmc.octoMap.key.KeySet;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.node.AbstractOccupancyOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.octoMap.pointCloud.ScanNode;
import us.ihmc.octoMap.tools.OcTreeKeyTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;

public abstract class AbstractOccupancyOcTreeBase<NODE extends AbstractOccupancyOcTreeNode<NODE>> extends AbstractOccupancyOcTree<NODE>
{
   protected boolean useBoundingBoxLimit; ///< use bounding box for queries (needs to be set)?
   protected final Point3d boundingBoxMin = new Point3d();
   protected final Point3d boundingBoxMax = new Point3d();
   protected final OcTreeKey boundingBoxMinKey = new OcTreeKey();
   protected final OcTreeKey boundingBoxMaxKey = new OcTreeKey();

   protected boolean useChangeDetection;
   /** Set of leaf keys (lowest level) which changed since last resetChangeDetection */
   protected KeyBoolMap changedKeys = new KeyBoolMap();

   private final ConcurrentLinkedQueue<List<OcTreeKey>> keyListPool = new ConcurrentLinkedQueue<>();

   public AbstractOccupancyOcTreeBase(double resolution)
   {
      super(resolution);
      useBoundingBoxLimit = false;
      useChangeDetection = false;
      initTemporaryFields();
   }

   /// Constructor to enable derived classes to change tree constants.
   /// This usually requires a re-implementation of some core tree-traversal functions as well!
   protected AbstractOccupancyOcTreeBase(double resolution, int tree_depth, int tree_max_val)
   {
      super(resolution, tree_depth, tree_max_val);
      useBoundingBoxLimit = false;
      useChangeDetection = false;
      initTemporaryFields();
   }

   public AbstractOccupancyOcTreeBase(AbstractOccupancyOcTreeBase<NODE> other)
   {
      super(other);
      useBoundingBoxLimit = other.useBoundingBoxLimit;
      boundingBoxMin.set(other.boundingBoxMin);
      boundingBoxMax.set(other.boundingBoxMax);
      boundingBoxMinKey.set(other.boundingBoxMinKey);
      boundingBoxMaxKey.set(other.boundingBoxMaxKey);
      changedKeys.putAll(other.changedKeys);
      useChangeDetection = other.useChangeDetection;
      initTemporaryFields();
   }

   private void initTemporaryFields()
   {
      for (int i = 0; i < 6; i++)
      {
         keyListPool.add(new ArrayList<>(1000000));
      }
   }

   public void insertPointCloud(PointCloud scan, Point3d sensorOrigin)
   {
      insertPointCloud(scan, sensorOrigin, -1.0, false, false);
   }

   public void insertPointCloud(PointCloud scan, Point3d sensorOrigin, boolean lazyEvaluation, boolean discretize)
   {
      insertPointCloud(scan, sensorOrigin, -1.0, lazyEvaluation, discretize);
   }

   /**
    * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
    * Special care is taken that each voxel
    * in the map is updated only once, and occupied nodes have a preference over free ones.
    * This avoids holes in the floor from mutual deletion and is more efficient than the plain
    * ray insertion in insertPointCloudRays().
    *
    * @note replaces insertScan()
    *
    * @param scan Pointcloud (measurement endpoints), in global reference frame
    * @param sensorOrigin measurement origin in global reference frame
    * @param maxRange maximum range for how long individual beams are inserted (default -1: complete beam)
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @param discretize whether the scan is discretized first into octree key cells (default: false).
    *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.*
    */
   public void insertPointCloud(PointCloud scan, Point3d sensorOrigin, double maxRange, boolean lazyEvaluation, boolean discretize)
   {
      List<OcTreeKey> freeCells = keyListPool.isEmpty() ? new ArrayList<>(1000000) : keyListPool.poll();
      List<OcTreeKey> occupiedCells = keyListPool.isEmpty() ? new ArrayList<>(1000000) : keyListPool.poll();
      freeCells.clear();
      occupiedCells.clear();

      if (discretize)
         computeDiscreteUpdate(scan, sensorOrigin, freeCells, occupiedCells, maxRange);
      else
         computeUpdate(scan, sensorOrigin, freeCells, occupiedCells, maxRange);

      // insert data into tree  -----------------------
      for (OcTreeKey key : occupiedCells)
         updateNode(key, true, lazyEvaluation, true);

      for (OcTreeKey key : freeCells)
         updateNode(key, false, lazyEvaluation, true);

      root.enablePointCloudModeRecursively(false);
      keyListPool.add(freeCells);
      keyListPool.add(occupiedCells);
   }

   public void insertPointCloud(PointCloud scan, Point3d sensorOrigin, RigidBodyTransform frameOrigin)
   {
      insertPointCloud(scan, sensorOrigin, frameOrigin, -1.0, false, false);
   }

   /**
   * Integrate a 3d scan (transform scan before tree update), parallelized with OpenMP.
   * Special care is taken that each voxel
   * in the map is updated only once, and occupied nodes have a preference over free ones.
   * This avoids holes in the floor from mutual deletion and is more efficient than the plain
   * ray insertion in insertPointCloudRays().
   *
   * @note replaces insertScan()
   *
   * @param scan Pointcloud (measurement endpoints) relative to frame origin
   * @param sensorOrigin origin of sensor relative to frame origin
   * @param frameOrigin origin of reference frame, determines transform to be applied to cloud and sensor origin
   * @param maxRange maximum range for how long individual beams are inserted (default -1: complete beam)
   * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
   *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
   * @param discretize whether the scan is discretized first into octree key cells (default: false).
   *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.*
   */
   public void insertPointCloud(PointCloud scan, Point3d sensorOrigin, RigidBodyTransform frameOrigin, double maxRange, boolean lazyEvaluation, boolean discretize)
   {
      // performs transformation to data and sensor origin first
      PointCloud transformedScan = new PointCloud(scan);
      transformedScan.transform(frameOrigin);
      Point3d transformed_sensorOrigin = new Point3d(sensorOrigin);
      frameOrigin.transform(transformed_sensorOrigin);
      insertPointCloud(transformedScan, transformed_sensorOrigin, maxRange, lazyEvaluation, discretize);
   }

   public void insertPointCloud(ScanNode scan)
   {
      insertPointCloud(scan, -1.0, false, false);
   }

   /**
   * Insert a 3d scan (given as a ScanNode) into the tree, parallelized with OpenMP.
   *
   * @note replaces insertScan
   *
   * @param scan ScanNode contains Pointcloud data and frame/sensor origin
   * @param maxRange maximum range for how long individual beams are inserted (default -1: complete beam)
   * @param lazyEvaluation whether the tree is left 'dirty' after the update (default: false).
   *   This speeds up the insertion by not updating inner nodes, but you need to call updateInnerOccupancy() when done.
   * @param discretize whether the scan is discretized first into octree key cells (default: false).
   *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.
   */
   public void insertPointCloud(ScanNode scan, double maxRange, boolean lazyEvaluation, boolean discretize)
   {
      // performs transformation to data and sensor origin first
      PointCloud cloud = scan.getScan();
      RigidBodyTransform frame_origin = new RigidBodyTransform(scan.getPose());
      frame_origin.invert();
      Vector3d tempVector = new Vector3d();
      scan.getPose().get(tempVector);
      Point3d sensorOrigin = new Point3d(tempVector);//frame_origin.inv().transform(scan.pose.trans()); // TODO Sylvain Double-check this transformation
      frame_origin.transform(sensorOrigin);
      insertPointCloud(cloud, sensorOrigin, frame_origin, maxRange, lazyEvaluation, discretize);
   }

   public void insertPointCloudRays(PointCloud scan, Point3d sensorOrigin)
   {
      insertPointCloudRays(scan, sensorOrigin, -1.0, false);
   }

   /**
    * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
    * This function simply inserts all rays of the point clouds as batch operation.
    * Discretization effects can lead to the deletion of occupied space, it is
    * usually recommended to use insertPointCloud() instead.
    *
    * @param scan Pointcloud (measurement endpoints), in global reference frame
    * @param sensorOrigin measurement origin in global reference frame
    * @param maxRange maximum range for how long individual beams are inserted (default -1: complete beam)
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    */
   public void insertPointCloudRays(PointCloud scan, Point3d sensorOrigin, double maxRange, boolean lazyEvaluation)
   {
      if (scan.size() < 1)
         return;

      for (int i = 0; i < scan.size(); i++)
      {
         Point3d point = new Point3d(scan.getPoint(i));
         KeyRay keyRay = keyrays.get(0);
         if (computeRayKeys(sensorOrigin, point, keyRay))
         {
            for (OcTreeKey key : keyRay)
            {
               updateNode(key, false, lazyEvaluation); // insert freespace measurement
            }
            updateNode(point, true, lazyEvaluation); // update endpoint to be occupied
         }
      }
   }

   public NODE setNodeValue(OcTreeKey key, float logOddsValue)
   {
      return setNodeValue(key, logOddsValue, false);
   }

   /**
    * Set log_odds value of voxel to logOddsValue. This only works if key is at the lowest
    * octree level
    *
    * @param key OcTreeKey of the NODE that is to be updated
    * @param logOddsValue value to be set as the log_odds value of the node
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE setNodeValue(OcTreeKey key, float logOddsValue, boolean lazyEvaluation)
   {
      // clamp log odds within range:
      logOddsValue = Math.min(Math.max(logOddsValue, minOccupancyLogOdds), maxOccupancyLogOdds);

      boolean createdRoot = false;
      if (root == null)
      {
         root = createRootNode();
         treeSize++;
         createdRoot = true;
      }
      return setNodeValueRecurs(root, createdRoot, key, 0, logOddsValue, lazyEvaluation);
   }

   public NODE setNodeValue(Point3d coordinate, float logOddsValue)
   {
      return setNodeValue(coordinate, logOddsValue, false);
   }

   /**
    * Set log_odds value of voxel to logOddsValue.
    * Looks up the OcTreeKey corresponding to the coordinate and then calls setNodeValue() with it.
    *
    * @param coordinate 3d coordinate of the NODE that is to be updated
    * @param logOddsValue value to be set as the log_odds value of the node
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE setNodeValue(Point3d coordinate, float logOddsValue, boolean lazyEvaluation)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(coordinate);
      if (key == null)
         return null;

      return setNodeValue(key, logOddsValue, lazyEvaluation);
   }

   public NODE setNodeValue(double x, double y, double z, float logOddsValue)
   {
      return setNodeValue(x, y, z, logOddsValue, false);
   }

   /**
    * Set log_odds value of voxel to logOddsValue.
    * Looks up the OcTreeKey corresponding to the coordinate and then calls setNodeValue() with it.
    *
    * @param x
    * @param y
    * @param z
    * @param logOddsValue value to be set as the log_odds value of the node
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE setNodeValue(double x, double y, double z, float logOddsValue, boolean lazyEvaluation)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(x, y, z);
      if (key == null)
         return null;

      return setNodeValue(key, logOddsValue, lazyEvaluation);
   }

   @Override
   public NODE updateNode(OcTreeKey key, float logOddsUpdate, boolean lazyEvaluation)
   {
      return updateNode(key, logOddsUpdate, lazyEvaluation, false);
   }

   /**
    * Manipulate log_odds value of a voxel by changing it by logOddsUpdate (relative).
    * This only works if key is at the lowest octree level
    *
    * @param key OcTreeKey of the NODE that is to be updated
    * @param logOddsUpdate value to be added (+) to log_odds value of node
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(OcTreeKey key, float logOddsUpdate, boolean lazyEvaluation, boolean enablePointCloudMode)
   {
      NODE leaf = search(key);

      if (leaf != null)
      {
         // early abort (no change will happen).
         // may cause an overhead in some configuration, but more often helps
         // no change: node already at threshold
         boolean reachedMaxThreshold = logOddsUpdate >= 0.0f && leaf.getLogOdds() >= maxOccupancyLogOdds;
         boolean reachedMinThreshold = logOddsUpdate <= 0.0f && leaf.getLogOdds() <= minOccupancyLogOdds;

         if (!leaf.isUpdatable() || reachedMaxThreshold || reachedMinThreshold)
            return leaf;
      }

      boolean createdRoot = false;
      if (root == null)
      {
         root = createRootNode();
         treeSize++;
         createdRoot = true;
      }

      root.enablePointCloudModeRecursively(enablePointCloudMode);

      return updateNodeRecurs(root, createdRoot, key, 0, logOddsUpdate, lazyEvaluation);
   }

   /**
    * Manipulate log_odds value of a voxel by changing it by logOddsUpdate (relative).
    * Looks up the OcTreeKey corresponding to the coordinate and then calls updateNode() with it.
    *
    * @param coordinate 3d coordinate of the NODE that is to be updated
    * @param logOddsUpdate value to be added (+) to log_odds value of node
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   @Override
   public NODE updateNode(Point3d coordinate, float logOddsUpdate, boolean lazyEvaluation)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(coordinate);
      if (key == null)
         return null;

      return updateNode(key, logOddsUpdate, lazyEvaluation);
   }

   public NODE updateNode(double x, double y, double z, float logOddsUpdate)
   {
      return updateNode(x, y, z, logOddsUpdate, false);
   }

   /**
    * Manipulate log_odds value of a voxel by changing it by logOddsUpdate (relative).
    * Looks up the OcTreeKey corresponding to the coordinate and then calls updateNode() with it.
    *
    * @param x
    * @param y
    * @param z
    * @param logOddsUpdate value to be added (+) to log_odds value of node
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(double x, double y, double z, float logOddsUpdate, boolean lazyEvaluation)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(x, y, z);
      if (key == null)
         return null;

      return updateNode(key, logOddsUpdate, lazyEvaluation);
   }

   @Override
   public NODE updateNode(OcTreeKey key, boolean occupied, boolean lazyEvaluation)
   {
      return updateNode(key, occupied, lazyEvaluation, false);
   }

   /**
    * Integrate occupancy measurement.
    *
    * @param key OcTreeKey of the NODE that is to be updated
    * @param occupied true if the node was measured occupied, else false
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(OcTreeKey key, boolean occupied, boolean lazyEvaluation, boolean enablePointCloudMode)
   {
      float logOdds = missUpdateLogOdds;
      if (occupied)
         logOdds = hitUpdateLogOdds;

      return updateNode(key, logOdds, lazyEvaluation, enablePointCloudMode);
   }

   /**
    * Integrate occupancy measurement.
    * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
    *
    * @param coordinate 3d coordinate of the NODE that is to be updated
    * @param occupied true if the node was measured occupied, else false
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   @Override
   public NODE updateNode(Point3d coordinate, boolean occupied, boolean lazyEvaluation)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(coordinate);
      if (key == null)
         return null;
      return updateNode(key, occupied, lazyEvaluation);
   }

   public NODE updateNode(double x, double y, double z, boolean occupied)
   {
      return updateNode(x, y, z, occupied, false);
   }

   /**
    * Integrate occupancy measurement.
    * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
    *
    * @param x
    * @param y
    * @param z
    * @param occupied true if the node was measured occupied, else false
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(double x, double y, double z, boolean occupied, boolean lazyEvaluation)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(x, y, z);
      if (key == null)
         return null;
      return updateNode(key, occupied, lazyEvaluation);
   }

   /**
    * Creates the maximum likelihood map by calling toMaxLikelihood on all
    * tree nodes, setting their occupancy to the corresponding occupancy thresholds.
    * This enables a very efficient compression if you call prune() afterwards.
    */
   @Override
   public void toMaxLikelihood()
   {
      if (root == null)
         return;

      // convert bottom up
      for (int depth = treeDepth; depth > 0; depth--)
      {
         toMaxLikelihoodRecurs(root, 0, depth);
      }

      // convert root
      nodeToMaxLikelihood(root);
   }

   public boolean insertRay(Point3d origin, Point3d end)
   {
      return insertRay(origin, end, -1.0, false);
   }

   /**
    * Insert one ray between origin and end into the tree.
    * integrateMissOnRay() is called for the ray, the end point is updated as occupied.
    * It is usually more efficient to insert complete pointcloudsm with insertPointCloud() or
    * insertPointCloudRays().
    *
    * @param origin origin of sensor in global coordinates
    * @param end endpoint of measurement in global coordinates
    * @param maxRange maximum range after which the raycast should be aborted
    * @param lazyEvaluation whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return success of operation
    */
   public boolean insertRay(Point3d origin, Point3d end, double maxRange, boolean lazyEvaluation)
   {
      Vector3d direction = new Vector3d();
      direction.sub(end, origin);
      double length = direction.length();

      // cut ray at maxrange
      if (maxRange > 0 && length > maxRange)
      {
         direction.scale(1.0 / length);
         Point3d new_end = new Point3d();
         new_end.scaleAdd(maxRange, direction, origin);
         return integrateMissOnRay(origin, new_end, lazyEvaluation);
      }
      // insert complete ray
      else
      {
         if (!integrateMissOnRay(origin, end, lazyEvaluation))
            return false;
         updateNode(end, true, lazyEvaluation); // insert hit cell
         return true;
      }
   }

   public boolean castRay(Point3d origin, Vector3d direction, Point3d end)
   {
      return castRay(origin, direction, end, false);
   }

   public boolean castRay(Point3d origin, Vector3d direction, Point3d end, boolean ignoreUnknownCells)
   {
      return castRay(origin, direction, end, ignoreUnknownCells, -1.0);
   }

   /**
    * Performs raycasting in 3d, similar to computeRay(). Can be called in parallel e.g. with OpenMP
    * for a speedup.
    *
    * A ray is cast from 'origin' with a given direction, the first non-free
    * cell is returned in 'end' (as center coordinate). This could also be the 
    * origin node if it is occupied or unknown. castRay() returns true if an occupied node
    * was hit by the raycast. If the raycast returns false you can search() the node at 'end' and
    * see whether it's unknown space.
    * 
    *
    * @param[in] origin starting coordinate of ray
    * @param[in] direction A vector pointing in the direction of the raycast (NOT a point in space). Does not need to be normalized.
    * @param[out] end returns the center of the last cell on the ray. If the function returns true, it is occupied.
    * @param[in] ignoreUnknownCells whether unknown cells are ignored (= treated as free). If false (default), the raycast aborts when an unknown cell is hit and returns false.
    * @param[in] maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
    * @return true if an occupied cell was hit, false if the maximum range or octree bounds are reached, or if an unknown node was hit.
    */
   public boolean castRay(Point3d origin, Vector3d direction, Point3d end, boolean ignoreUnknownCells, double maxRange)
   {
      /// ----------  see OcTreeBase::computeRayKeys  -----------

      // Initialization phase -------------------------------------------------------
      OcTreeKey current_key = convertCartesianCoordinateToKey(origin);
      if (current_key == null)
      {
         PrintTools.warn(this, "Coordinates out of bounds during ray casting");
         return false;
      }

      NODE startingNode = search(current_key);
      if (startingNode != null)
      {
         if (isNodeOccupied(startingNode))
         {
            // Occupied node found at origin 
            // (need to convert from key, since origin does not need to be a voxel center)
            keyToCoord(current_key, end);
            return true;
         }
      }
      else if (!ignoreUnknownCells)
      {
         keyToCoord(current_key, end);
         return false;
      }

      direction = new Vector3d(direction);
      direction.normalize();
      boolean max_range_set = maxRange > 0.0;

      double[] originArray = new double[3];
      origin.get(originArray);
      double[] endArray = new double[3];
      end.get(endArray);
      double[] directionArray = new double[3];
      direction.get(directionArray);
      int[] step = new int[3];
      double[] tMax = new double[3];
      double[] tDelta = new double[3];

      for (int i = 0; i < 3; ++i)
      {
         // compute step direction
         if (directionArray[i] > 0.0)
            step[i] = 1;
         else if (directionArray[i] < 0.0)
            step[i] = -1;
         else
            step[i] = 0;

         // compute tMax, tDelta
         if (step[i] != 0)
         {
            // corner point of voxel (in direction of ray)
            double voxelBorder = keyToCoord(current_key.k[i]);
            voxelBorder += step[i] * resolution * 0.5;

            tMax[i] = (voxelBorder - originArray[i]) / directionArray[i];
            tDelta[i] = resolution / Math.abs(directionArray[i]);
         }
         else
         {
            tMax[i] = Double.POSITIVE_INFINITY;
            tDelta[i] = Double.POSITIVE_INFINITY;
         }
      }

      if (step[0] == 0 && step[1] == 0 && step[2] == 0)
      {
         PrintTools.error(this, "Raycasting in direction (0,0,0) is not possible!");
         return false;
      }

      // for speedup:
      double maxrange_sq = maxRange * maxRange;

      // Incremental phase  ---------------------------------------------------------

      boolean done = false;

      while (!done)
      {
         int dim;

         // find minimum tMax:
         if (tMax[0] < tMax[1])
         {
            if (tMax[0] < tMax[2])
               dim = 0;
            else
               dim = 2;
         }
         else
         {
            if (tMax[1] < tMax[2])
               dim = 1;
            else
               dim = 2;
         }

         // check for overflow:
         if (step[dim] < 0 && current_key.k[dim] == 0 || step[dim] > 0 && current_key.k[dim] == 2 * treeMaximumValue - 1)
         {
            PrintTools.warn(this, "Coordinate hit bounds in dim " + dim + ", aborting raycast");
            // return border point nevertheless:
            keyToCoord(current_key, end);
            return false;
         }

         // advance in direction "dim"
         current_key.k[dim] += step[dim];
         tMax[dim] += tDelta[dim];

         // generate world coords from key
         keyToCoord(current_key, end);

         // check for maxrange:
         if (max_range_set)
         {
            double dist_from_origin_sq = 0.0;
            for (int j = 0; j < 3; j++)
            {
               dist_from_origin_sq += (endArray[j] - originArray[j]) * (endArray[j] - originArray[j]);
            }
            if (dist_from_origin_sq > maxrange_sq)
               return false;

         }

         NODE currentNode = search(current_key);
         if (currentNode != null)
         {
            if (isNodeOccupied(currentNode))
            {
               done = true;
               break;
            }
            // otherwise: node is free and valid, raycasting continues
         }
         else if (!ignoreUnknownCells)
         { // no node found, this usually means we are in "unknown" areas
            return false;
         }
      } // end while

      return true;
   }

   public boolean getRayIntersection(Point3d origin, Vector3d direction, Point3d center, Point3d intersection)
   {
      return getRayIntersection(origin, direction, center, intersection, 0.0);
   }

   /**
    * Retrieves the entry point of a ray into a voxel. This is the closest intersection point of the ray
    * originating from origin and a plane of the axis aligned cube.
    * 
    * @param[in] origin Starting point of ray
    * @param[in] direction A vector pointing in the direction of the raycast. Does not need to be normalized.
    * @param[in] center The center of the voxel where the ray terminated. This is the output of castRay.
    * @param[out] intersection The entry point of the ray into the voxel, on the voxel surface.
    * @param[in] delta A small increment to avoid ambiguity of beeing exactly on a voxel surface. A positive value will get the point out of the hit voxel, while a negative valuewill get it inside.
    * @return Whether or not an intesection point has been found. Either, the ray never cross the voxel or the ray is exactly parallel to the only surface it intersect.
    */
   public boolean getRayIntersection(Point3d origin, Vector3d direction, Point3d center, Point3d intersection, double delta)
   {
      // We only need three normals for the six planes
      Vector3d normalX = new Vector3d(1, 0, 0);
      Vector3d normalY = new Vector3d(0, 1, 0);
      Vector3d normalZ = new Vector3d(0, 0, 1);

      // One point on each plane, let them be the center for simplicity
      Vector3d pointXNeg = new Vector3d(center.getX() - resolution / 2.0, center.getY(), center.getZ());
      Vector3d pointXPos = new Vector3d(center.getX() + resolution / 2.0, center.getY(), center.getZ());
      Vector3d pointYNeg = new Vector3d(center.getX(), center.getY() - resolution / 2.0, center.getZ());
      Vector3d pointYPos = new Vector3d(center.getX(), center.getY() + resolution / 2.0, center.getZ());
      Vector3d pointZNeg = new Vector3d(center.getX(), center.getY(), center.getZ() - resolution / 2.0);
      Vector3d pointZPos = new Vector3d(center.getX(), center.getY(), center.getZ() + resolution / 2.0);

      double lineDotNormal = 0.0;
      double d = 0.0;
      double outD = Double.POSITIVE_INFINITY;
      Point3d intersect = new Point3d();
      boolean found = false;

      Vector3d tempVector = new Vector3d();

      // Find the intersection (if any) with each place
      // Line dot normal will be zero if they are parallel, in which case no intersection can be the entry one
      // if there is an intersection does it occur in the bounded plane of the voxel
      // if yes keep only the closest (smallest distance to sensor origin).
      if ((lineDotNormal = normalX.dot(direction)) != 0.0)
      {
         tempVector.sub(pointXNeg, origin);
         d = tempVector.dot(normalX) / lineDotNormal;
         intersect.scaleAdd(d, direction, origin);
         if (!(intersect.getY() < pointYNeg.getY() - 1e-6 || intersect.getY() > pointYPos.getY() + 1e-6 || intersect.getZ() < pointZNeg.getZ() - 1e-6
               || intersect.getZ() > pointZPos.getZ() + 1e-6))
         {
            outD = Math.min(outD, d);
            found = true;
         }

         tempVector.sub(pointXPos, origin);
         d = tempVector.dot(normalX) / lineDotNormal;
         intersect.scaleAdd(d, direction, origin);
         if (!(intersect.getY() < pointYNeg.getY() - 1e-6 || intersect.getY() > pointYPos.getY() + 1e-6 || intersect.getZ() < pointZNeg.getZ() - 1e-6
               || intersect.getZ() > pointZPos.getZ() + 1e-6))
         {
            outD = Math.min(outD, d);
            found = true;
         }
      }

      if ((lineDotNormal = normalY.dot(direction)) != 0.0)
      {
         tempVector.sub(pointYNeg, origin);
         d = tempVector.dot(normalY) / lineDotNormal;
         intersect.scaleAdd(d, direction, origin);
         if (!(intersect.getX() < pointXNeg.getX() - 1e-6 || intersect.getX() > pointXPos.getX() + 1e-6 || intersect.getZ() < pointZNeg.getZ() - 1e-6
               || intersect.getZ() > pointZPos.getZ() + 1e-6))
         {
            outD = Math.min(outD, d);
            found = true;
         }

         tempVector.sub(pointYPos, origin);
         d = tempVector.dot(normalY) / lineDotNormal;
         intersect.scaleAdd(d, direction, origin);
         if (!(intersect.getX() < pointXNeg.getX() - 1e-6 || intersect.getX() > pointXPos.getX() + 1e-6 || intersect.getZ() < pointZNeg.getZ() - 1e-6
               || intersect.getZ() > pointZPos.getZ() + 1e-6))
         {
            outD = Math.min(outD, d);
            found = true;
         }
      }

      if ((lineDotNormal = normalZ.dot(direction)) != 0.0)
      {
         tempVector.sub(pointZNeg, origin);
         d = tempVector.dot(normalZ) / lineDotNormal;
         intersect.scaleAdd(d, direction, origin);
         if (!(intersect.getX() < pointXNeg.getX() - 1e-6 || intersect.getX() > pointXPos.getX() + 1e-6 || intersect.getY() < pointYNeg.getY() - 1e-6
               || intersect.getY() > pointYPos.getY() + 1e-6))
         {
            outD = Math.min(outD, d);
            found = true;
         }

         tempVector.sub(pointZPos, origin);
         d = tempVector.dot(normalZ) / lineDotNormal;
         intersect.scaleAdd(d, direction, origin);
         if (!(intersect.getX() < pointXNeg.getX() - 1e-6 || intersect.getX() > pointXPos.getX() + 1e-6 || intersect.getY() < pointYNeg.getY() - 1e-6
               || intersect.getY() > pointYPos.getY() + 1e-6))
         {
            outD = Math.min(outD, d);
            found = true;
         }
      }

      // Substract (add) a fraction to ensure no ambiguity on the starting voxel
      // Don't start on a bondary.
      if (found)
         intersection.scaleAdd(outD + delta, direction, origin);

      return found;
   }

   public boolean getNormals(Point3d point, List<Vector3d> normals)
   {
      return getNormals(point, normals, true);
   }

   /**
      * Performs a step of the marching cubes surface reconstruction algorithm
      * to retrieve the normal of the triangles that fall in the cube
      * formed by the voxels located at the vertex of a given voxel.
      *
      * @param[in] voxel for which retrieve the normals
      * @param[out] triangles normals
      * @param[in] unknownStatus consider unknown cells as free (false) or occupied (default, true).
      * @return True if the input voxel is known in the occupancy grid, and false if it is unknown.
      */
   public boolean getNormals(Point3d voxel, List<Vector3d> normals, boolean unknownStatus)
   {
      OcTreeKey initKey = convertCartesianCoordinateToKey(voxel);
      if (initKey == null)
      {
         PrintTools.error(this, "Voxel out of bounds");
         return false;
      }

      return getNormals(initKey, normals, unknownStatus);
   }
   
   public boolean getNormals(OcTreeKey key, List<Vector3d> normals)
   {
      return getNormals(key, normals, true);
   }

   public boolean getNormals(OcTreeKey key, List<Vector3d> normals, boolean unknownStatus)
   {
      normals.clear();
      // OCTOMAP_WARNING("Normal for %f, %f, %f\n", point.x(), point.y(), point.z());

      int[] vertexValues = new int[8];

      OcTreeKey currentKey = new OcTreeKey();
      NODE currentNode;

      // There is 8 neighbouring sets
      // The current cube can be at any of the 8 vertex
      int[][] xIndex = new int[][] {{1, 1, 0, 0}, {1, 1, 0, 0}, {0, 0, - 1, -1}, {0, 0, - 1, -1}};
      int[][] yIndex = new int[][] {{1, 0, 0, 1}, {0, -1, -1, 0}, {0, -1, -1, 0}, {1, 0, 0, 1}};
      int[][] zIndex = new int[][] {{0, 1}, {-1, 0}};

      // Iterate over the 8 neighboring sets
      for (int m = 0; m < 2; ++m)
      {
         for (int l = 0; l < 4; ++l)
         {
            int k = 0;
            // Iterate over the cubes
            for (int j = 0; j < 2; ++j)
            {
               for (int i = 0; i < 4; ++i)
               {
                  currentKey.k[0] = key.k[0] + xIndex[l][i];
                  currentKey.k[1] = key.k[1] + yIndex[l][i];
                  currentKey.k[2] = key.k[2] + zIndex[m][j];
                  currentNode = search(currentKey);

                  if (currentNode != null)
                  {
                     vertexValues[k] = isNodeOccupied(currentNode) ? 1 : 0;

                     // point3d coord = this->keyToCoord(current_key);
                     // OCTOMAP_WARNING_STR("vertex " << k << " at " << coord << "; value " << vertex_values[k]);
                  }
                  else
                  {
                     // Occupancy of unknown cells
                     vertexValues[k] = unknownStatus ? 1 : 0;
                  }
                  ++k;
               }
            }

            int cubeIndex = 0;
            if (vertexValues[0] != 0) cubeIndex |= 1;
            if (vertexValues[1] != 0) cubeIndex |= 2;
            if (vertexValues[2] != 0) cubeIndex |= 4;
            if (vertexValues[3] != 0) cubeIndex |= 8;
            if (vertexValues[4] != 0) cubeIndex |= 16;
            if (vertexValues[5] != 0) cubeIndex |= 32;
            if (vertexValues[6] != 0) cubeIndex |= 64;
            if (vertexValues[7] != 0) cubeIndex |= 128;

            // OCTOMAP_WARNING_STR("cubde_index: " << cube_index);

            // All vertices are occupied or free resulting in no normal
            if (edgeTable[cubeIndex] == 0)
               continue; //return true;

            // No interpolation is done yet, we use vertexList in <MCTables.h>.
            for (int i = 0; triTable[cubeIndex][i] != -1; i += 3)
            {
               Point3d p1 = new Point3d(vertexList[triTable[cubeIndex][i]]);
               Point3d p2 = new Point3d(vertexList[triTable[cubeIndex][i + 1]]);
               Point3d p3 = new Point3d(vertexList[triTable[cubeIndex][i + 2]]);
               Vector3d v1 = new Vector3d(); //p2 - p1;
               Vector3d v2 = new Vector3d(); //p3 - p1;
               Vector3d v3 = new Vector3d();
               v1.sub(p2, p1);
               v2.sub(p3, p1);
               v3.cross(v1, v2);
               v3.normalize();

               // OCTOMAP_WARNING("Vertex p1 %f, %f, %f\n", p1.x(), p1.y(), p1.z());
               // OCTOMAP_WARNING("Vertex p2 %f, %f, %f\n", p2.x(), p2.y(), p2.z());
               // OCTOMAP_WARNING("Vertex p3 %f, %f, %f\n", p3.x(), p3.y(), p3.z());

               // Right hand side cross product to retrieve the normal in the good
               // direction (pointing to the free nodes).
               normals.add(v3);
            }
         }
      }

      return true;
   }

   //-- set BBX limit (limits tree updates to this bounding box)

   ///  use or ignore BBX limit (default: ignore)
   public void useBoundingBoxLimit(boolean enable)
   {
      useBoundingBoxLimit = enable;
   }

   public boolean isUsingBoundingBox()
   {
      return useBoundingBoxLimit;
   }

   /// sets the minimum for a query bounding box to use
   public void setBoundingBoxMin(Point3d min)
   {
      OcTreeKey newKey = convertCartesianCoordinateToKey(boundingBoxMin);
      if (newKey == null)
      {
         PrintTools.error(this, "ERROR while generating bbx min key.");
      }
      
      boundingBoxMin.set(min);
      boundingBoxMinKey.set(newKey);
   }

   /// sets the maximum for a query bounding box to use
   public void setBoundingBoxMax(Point3d max)
   {
      OcTreeKey newKey = convertCartesianCoordinateToKey(boundingBoxMax);
      if (newKey == null)
      {
         PrintTools.error(this, "ERROR while generating bbx max key.");
      }

      boundingBoxMax.set(max);
      boundingBoxMaxKey.set(newKey);
   }

   /// @return the currently set minimum for bounding box queries, if set
   public Point3d getBoundingBoxMin()
   {
      return boundingBoxMin;
   }

   /// @return the currently set maximum for bounding box queries, if set
   public Point3d getBoundingBoxMax()
   {
      return boundingBoxMax;
   }

   public Point3d getBoundingBoxBounds()
   {
      Point3d obj_bounds = new Point3d();
      obj_bounds.sub(boundingBoxMax, boundingBoxMin);
      obj_bounds.scale(0.5);
      return obj_bounds;
   }

   public Point3d getBoundingBoxCenter()
   {
      Point3d center = getBoundingBoxBounds();
      center.add(boundingBoxMin);
      return center;
   }

   /// @return true if point is in the currently set bounding box
   public boolean isInBoundingBox(Point3d p)
   {
      return p.getX() >= boundingBoxMin.getX() && p.getY() >= boundingBoxMin.getY() && p.getZ() >= boundingBoxMin.getZ() && p.getX() <= boundingBoxMax.getX()
            && p.getY() <= boundingBoxMax.getY() && p.getZ() <= boundingBoxMax.getZ();
   }

   /// @return true if key is in the currently set bounding box
   public boolean isInBoundingBox(OcTreeKey key)
   {
      return key.k[0] >= boundingBoxMinKey.k[0] && key.k[1] >= boundingBoxMinKey.k[1] && key.k[2] >= boundingBoxMinKey.k[2] && key.k[0] <= boundingBoxMaxKey.k[0]
            && key.k[1] <= boundingBoxMaxKey.k[1] && key.k[2] <= boundingBoxMaxKey.k[2];
   }

   //-- change detection on occupancy:
   /// track or ignore changes while inserting scans (default: ignore)
   public void enableChangeDetection(boolean enable)
   {
      useChangeDetection = enable;
   }

   public boolean isChangeDetectionEnabled()
   {
      return useChangeDetection;
   }

   /// Reset the set of changed keys. Call this after you obtained all changed nodes.
   public void resetChangeDetection()
   {
      changedKeys.clear();
   }

   /// Number of changes since last reset.
   public int numberOfChangesDetected()
   {
      return changedKeys.size();
   }

   public KeyBoolMap getChangedKeys()
   {
      return changedKeys;
   }

   /**
    * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
    * integration at once. Here, occupied nodes have a preference over free
    * ones.
    *
    * @param scan point cloud measurement to be integrated
    * @param origin origin of the sensor for ray casting
    * @param freeCells keys of nodes to be cleared
    * @param occupiedCells keys of nodes to be marked occupied
    * @param maxrange maximum range for raycasting (-1: unlimited)
    */
   public void computeUpdate(PointCloud scan, Point3d origin, KeySet freeCells, KeySet occupiedCells, double maxrange)
   {
      for (int i = 0; i < scan.size(); ++i)
      {
         Point3d point = new Point3d(scan.getPoint(i));
         KeyRay keyray = getKeyRay(i);

         Vector3d direction = new Vector3d();
         direction.sub(point, origin);
         double length = direction.length();

         if (!useBoundingBoxLimit)
         { // no BBX specified
            if (maxrange < 0.0 || length <= maxrange)
            { // is not maxrange meas.
                 // free cells
               if (computeRayKeys(origin, point, keyray))
               {
                  freeCells.addAll(keyray);
               }
               // occupied endpoint
               OcTreeKey key = convertCartesianCoordinateToKey(point);
               if (key != null)
                  occupiedCells.add(key);
            }
            else
            { // user set a maxrange and length is above
               Point3d new_end = new Point3d();
               new_end.scaleAdd(maxrange / length, direction, origin);
               if (computeRayKeys(origin, new_end, keyray))
                  freeCells.addAll(keyray);
            } // end if maxrange
         }
         else
         { // BBX was set
              // endpoint in bbx and not maxrange?
            if (isInBoundingBox(point) && (maxrange < 0.0 || length <= maxrange))
            {
               // occupied endpoint
               OcTreeKey key = convertCartesianCoordinateToKey(point);
               if (key != null)
                  occupiedCells.add(key);

               // update freespace, break as soon as bbx limit is reached
               if (computeRayKeys(origin, point, keyray))
               {
                  ListIterator<OcTreeKey> reverseIterator = keyray.reverseIterator();
                  while (reverseIterator.hasPrevious())
                  {
                     OcTreeKey currentKey = reverseIterator.previous();
                     if (isInBoundingBox(currentKey))
                        freeCells.add(currentKey);
                     else
                        break;
                  }
               } // end if compute ray
            } // end if in BBX and not maxrange
         } // end bbx case

      } // end for all points, end of parallel OMP loop

      // prefer occupied cells over free ones (and make sets disjunct)
      freeCells.removeAll(occupiedCells);
   }

   /**
    * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
    * integration at once. By using lists, this method is faster than {@link #computeDiscreteUpdate(PointCloud, Point3d, KeySet, KeySet, double)}.
    * However, duplicates are not checked, freeCells can contain key contained in occupiedCells.
    *
    * @param scan point cloud measurement to be integrated
    * @param origin origin of the sensor for ray casting
    * @param freeCells keys of nodes to be cleared
    * @param occupiedCells keys of nodes to be marked occupied
    * @param maxrange maximum range for raycasting (-1: unlimited)
    */
   public void computeUpdate(PointCloud scan, Point3d origin, List<OcTreeKey> freeCells, List<OcTreeKey> occupiedCells, double maxrange)
   {
      for (int i = 0; i < scan.size(); ++i)
      {
         Point3d point = new Point3d(scan.getPoint(i));
         KeyRay keyray = keyrays.get(0);

         Vector3d direction = new Vector3d();
         direction.sub(point, origin);
         double length = direction.length();

         if (!useBoundingBoxLimit)
         { // no BBX specified
            if (maxrange < 0.0 || length <= maxrange)
            { // is not maxrange meas.
                 // free cells
               if (computeRayKeys(origin, point, keyray))
                  freeCells.addAll(keyray);
               // occupied endpoint
               OcTreeKey key = convertCartesianCoordinateToKey(point);
               if (key != null)
                  occupiedCells.add(key);
            }
            else
            { // user set a maxrange and length is above
               Point3d new_end = new Point3d();
               new_end.scaleAdd(maxrange / length, direction, origin);
               if (computeRayKeys(origin, new_end, keyray))
                  freeCells.addAll(keyray);
            } // end if maxrange
         }
         else
         { // BBX was set
              // endpoint in bbx and not maxrange?
            if (isInBoundingBox(point) && (maxrange < 0.0 || length <= maxrange))
            {
               // occupied endpoint
               OcTreeKey key = convertCartesianCoordinateToKey(point);
               if (key != null)
                  occupiedCells.add(key);

               // update freespace, break as soon as bbx limit is reached
               if (computeRayKeys(origin, point, keyray))
               {
                  ListIterator<OcTreeKey> reverseIterator = keyray.reverseIterator();
                  while (reverseIterator.hasPrevious())
                  {
                     OcTreeKey currentKey = reverseIterator.previous();
                     if (isInBoundingBox(currentKey))
                        freeCells.add(currentKey);
                     else
                        break;
                  }
               } // end if compute ray
            } // end if in BBX and not maxrange
         } // end bbx case

      } // end for all points, end of parallel OMP loop
   }

   protected KeyRay getKeyRay(int index)
   {
      while(keyrays.size() <= index)
         keyrays.add(new KeyRay(1000));
      return keyrays.get(index);
   }

   /**
    * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
    * integration at once. Here, occupied nodes have a preference over free
    * ones. This function first discretizes the scan with the octree grid, which results
    * in fewer raycasts (=speedup) but a slightly different result than computeUpdate().
    *
    * @param scan point cloud measurement to be integrated
    * @param origin origin of the sensor for ray casting
    * @param freeCells keys of nodes to be cleared
    * @param occupiedCells keys of nodes to be marked occupied
    * @param maxrange maximum range for raycasting (-1: unlimited)
    */
   public void computeDiscreteUpdate(PointCloud scan, Point3d origin, List<OcTreeKey> freeCells, List<OcTreeKey> occupiedCells, double maxrange)
   {
      PointCloud discretePC = new PointCloud();
      KeySet endpoints = new KeySet();

      for (int i = 0; i < scan.size(); ++i)
      {
         OcTreeKey key = convertCartesianCoordinateToKey(scan.getPoint(i));
         if (endpoints.add(key)) // insertion took place => key was not in set
            discretePC.add(keyToCoord(key));
      }

      computeUpdate(discretePC, origin, freeCells, occupiedCells, maxrange);
   }

   /**
    * Updates the occupancy of all inner nodes to reflect their children's occupancy.
    * If you performed batch-updates with lazy evaluation enabled, you must call this
    * before any queries to ensure correct multi-resolution behavior.
    **/
   public void updateInnerOccupancy()
   {
      if (root != null)
         updateInnerOccupancyRecurs(root, 0);
   }

   /**
    * Integrate a "hit" measurement according to the tree's sensor model
    * @param occupancyNode
    */
   public void integrateHit(NODE occupancyNode)
   {
      updateNodeLogOdds(occupancyNode, hitUpdateLogOdds);
   }

   /**
    * Integrate a "miss" measurement according to the tree's sensor model
    * @param occupancyNode
    */
   public void integrateMiss(NODE occupancyNode)
   {
      updateNodeLogOdds(occupancyNode, missUpdateLogOdds);
   }

   /**
    * Update logodds value of node by adding to the current value.
    * @param occupancyNode
    * @param update
    */
   public void updateNodeLogOdds(NODE occupancyNode, float update)
   {
      if (occupancyNode.isUpdatable())
      {
         occupancyNode.addValue(update);
         if (occupancyNode.getLogOdds() < minOccupancyLogOdds)
         {
            occupancyNode.setLogOdds(minOccupancyLogOdds);
            return;
         }
         if (occupancyNode.getLogOdds() > maxOccupancyLogOdds)
         {
            occupancyNode.setLogOdds(maxOccupancyLogOdds);
         }
      }
      occupancyNode.setHasBeenUpdated(true);
   }

   /**
    * Converts the node to the maximum likelihood occupancy value according to the tree's parameter for min/max "occupancy"
    * @param occupancyNode
    */
   public void nodeToMaxLikelihood(NODE occupancyNode)
   {
      if (isNodeOccupied(occupancyNode))
         occupancyNode.setLogOdds(maxOccupancyLogOdds);
      else
         occupancyNode.setLogOdds(minOccupancyLogOdds);
   }

   protected boolean integrateMissOnRay(Point3d origin, Point3d end)
   {
      return integrateMissOnRay(origin, end, false);
   }

   /**
    * Traces a ray from origin to end and updates all voxels on the
    *  way as free.  The volume containing "end" is not updated.
    */
   protected boolean integrateMissOnRay(Point3d origin, Point3d end, boolean lazyEvaluation)
   {
      KeyRay keyRay = keyrays.get(0);
      if (!computeRayKeys(origin, end, keyRay))
      {
         return false;
      }

      for (OcTreeKey key : keyRay)
      {
         updateNode(key, false, lazyEvaluation); // insert freespace measurement
      }

      return true;
   }

   // recursive calls ----------------------------

   protected NODE updateNodeRecurs(NODE node, boolean nodeJustCreated, OcTreeKey key, int depth, float logOddsUpdate)
   {
      return updateNodeRecurs(node, nodeJustCreated, key, depth, logOddsUpdate, false);
   }

   protected NODE updateNodeRecurs(NODE node, boolean nodeJustCreated, OcTreeKey key, int depth, float logOddsUpdate, boolean lazyEvaluation)
   {
      boolean createdNode = false;

      if (node == null)
         throw new RuntimeException("The given node is null.");

      // follow down to last level
      if (depth < treeDepth)
      {
         int pos = OcTreeKeyTools.computeChildIndex(key, treeDepth - 1 - depth);
         if (!OcTreeNodeTools.nodeChildExists(node, pos))
         {
            // child does not exist, but maybe it's a pruned node?
            if (!node.hasAtLeastOneChild() && !nodeJustCreated)
            {
               // current node does not have children AND it is not a new node 
               // -> expand pruned node
               expandNode(node);
            }
            else
            {
               // not a pruned node, create requested child
               createNodeChild(node, pos);
               createdNode = true;
            }
         }

         if (lazyEvaluation)
            return updateNodeRecurs(OcTreeNodeTools.getNodeChild(node, pos), createdNode, key, depth + 1, logOddsUpdate, lazyEvaluation);
         else
         {
            NODE retval = updateNodeRecurs(OcTreeNodeTools.getNodeChild(node, pos), createdNode, key, depth + 1, logOddsUpdate, lazyEvaluation);
            // prune node if possible, otherwise set own probability
            // note: combining both did not lead to a speedup!
            if (pruneNode(node))
            {
               // return pointer to current parent (pruned), the just updated node no longer exists
               retval = node;
            }
            else
            {
               node.updateOccupancyChildren();
            }

            return retval;
         }
      }
      else // at last level, update node, end of recursion
      {
         if (useChangeDetection)
         {
            boolean occBefore = isNodeOccupied(node);
            updateNodeLogOdds(node, logOddsUpdate);

            if (nodeJustCreated)
            { // new node
               changedKeys.put(key, true);
            }
            else if (occBefore != isNodeOccupied(node))
            { // occupancy changed, track it
               Boolean changedKeyValue = changedKeys.get(key);
               if (changedKeyValue == null)
                  changedKeys.put(key, false);
               else if (changedKeyValue == false)
                  changedKeys.remove(key);
            }
         }
         else
         {
            updateNodeLogOdds(node, logOddsUpdate);
         }
         return node;
      }
   }

   protected NODE setNodeValueRecurs(NODE node, boolean nodeJustCreated, OcTreeKey key, int depth, float logOddsValue)
   {
      return setNodeValueRecurs(node, nodeJustCreated, key, depth, logOddsValue, false);
   }

   protected NODE setNodeValueRecurs(NODE node, boolean nodeJustCreated, OcTreeKey key, int depth, float logOddsValue, boolean lazyEvaluation)
   {
      boolean created_node = false;

      if (node == null)
         throw new RuntimeException("The given node is null.");

      // follow down to last level
      if (depth < treeDepth)
      {
         int pos = OcTreeKeyTools.computeChildIndex(key, treeDepth - 1 - depth);
         if (!OcTreeNodeTools.nodeChildExists(node, pos))
         {
            // child does not exist, but maybe it's a pruned node?
            if (!node.hasAtLeastOneChild() && !nodeJustCreated)
            {
               // current node does not have children AND it is not a new node
               // -> expand pruned node
               expandNode(node);
            }
            else
            {
               // not a pruned node, create requested child
               createNodeChild(node, pos);
               created_node = true;
            }
         }

         if (lazyEvaluation)
            return setNodeValueRecurs(OcTreeNodeTools.getNodeChild(node, pos), created_node, key, depth + 1, logOddsValue, lazyEvaluation);
         else
         {
            NODE retval = setNodeValueRecurs(OcTreeNodeTools.getNodeChild(node, pos), created_node, key, depth + 1, logOddsValue, lazyEvaluation);
            // prune node if possible, otherwise set own probability
            // note: combining both did not lead to a speedup!
            if (pruneNode(node))
            {
               // return pointer to current parent (pruned), the just updated node no longer exists
               retval = node;
            }
            else
            {
               node.updateOccupancyChildren();
            }

            return retval;
         }
      }
      else // at last level, update node, end of recursion
      {
         if (useChangeDetection)
         {
            boolean occBefore = isNodeOccupied(node);
            node.setLogOdds(logOddsValue);

            if (nodeJustCreated)
            { // new node
               changedKeys.put(key, true);
            }
            else if (occBefore != isNodeOccupied(node))
            { // occupancy changed, track it
               Boolean keyChangedValue = changedKeys.get(key);
               if (keyChangedValue == null)
                  changedKeys.put(key, false);
               else if (keyChangedValue == false)
                  changedKeys.remove(key);
            }
         }
         else
         {
            node.setLogOdds(logOddsValue);
         }
         return node;
      }
   }

   protected void updateInnerOccupancyRecurs(NODE node, int depth)
   {
      if (node == null)
         throw new RuntimeException("The given node is null.");

      // only recurse and update for inner nodes:
      if (node.hasAtLeastOneChild())
      {
         // return early for last level:
         if (depth < treeDepth)
         {
            for (int i = 0; i < 8; i++)
            {
               if (OcTreeNodeTools.nodeChildExists(node, i))
               {
                  updateInnerOccupancyRecurs(OcTreeNodeTools.getNodeChild(node, i), depth + 1);
               }
            }
         }
         node.updateOccupancyChildren();
      }
   }

   protected void toMaxLikelihoodRecurs(NODE node, int depth, int max_depth)
   {
      if (node == null)
         throw new RuntimeException("The given node is null.");

      if (depth < max_depth)
      {
         for (int i = 0; i < 8; i++)
         {
            if (OcTreeNodeTools.nodeChildExists(node, i))
            {
               toMaxLikelihoodRecurs(OcTreeNodeTools.getNodeChild(node, i), depth + 1, max_depth);
            }
         }
      }
      else
      { // max level reached
         nodeToMaxLikelihood(node);
      }
   }
}