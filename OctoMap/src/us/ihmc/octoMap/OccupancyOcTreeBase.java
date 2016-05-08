package us.ihmc.octoMap;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.OcTreeKey.KeyBoolMap;
import us.ihmc.octoMap.OcTreeKey.KeySet;
import us.ihmc.octoMap.ScanGraph.ScanNode;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class OccupancyOcTreeBase<V, NODE extends OcTreeDataNode<V, NODE>> extends AbstractOccupancyOcTree<V, NODE>
{
   protected boolean use_bbx_limit;  ///< use bounding box for queries (needs to be set)?
   protected Point3d bbx_min = new Point3d();
   protected Point3d bbx_max = new Point3d();
   protected OcTreeKey bbx_min_key = new OcTreeKey();
   protected OcTreeKey bbx_max_key = new OcTreeKey();

   protected boolean use_change_detection;
   /// Set of leaf keys (lowest level) which changed since last resetChangeDetection
   protected KeyBoolMap changed_keys = new KeyBoolMap();

   public OccupancyOcTreeBase(double resolution)
   {
      super(resolution);
      use_bbx_limit = false;
      use_change_detection = false;
   }

   /// Constructor to enable derived classes to change tree constants.
   /// This usually requires a re-implementation of some core tree-traversal functions as well!
   protected OccupancyOcTreeBase(double resolution, int tree_depth, int tree_max_val)
   {
      super(resolution, tree_depth, tree_max_val);
      use_bbx_limit = false;
      use_change_detection = false;
   }


   public OccupancyOcTreeBase(OccupancyOcTreeBase<V, NODE> other)
   {
      super(other);
      use_bbx_limit = other.use_bbx_limit;
      bbx_min.set(other.bbx_min);
      bbx_max.set(other.bbx_max);
      bbx_min_key.set(other.bbx_min_key);
      bbx_max_key.set(other.bbx_max_key);
      changed_keys.putAll(other.changed_keys);
      use_change_detection = other.use_change_detection;
   }

   public void insertPointCloud(Pointcloud scan, Point3d sensor_origin);

   {
      insertPointCloud(scan, sensor_origin, -1.0, false, false);
   }

   public void insertPointCloud(Pointcloud scan, Point3d sensor_origin, boolean lazy_eval, boolean discretize)
   {
      insertPointCloud(scan, sensor_origin, -1.0, lazy_eval, discretize);
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
    * @param sensor_origin measurement origin in global reference frame
    * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @param discretize whether the scan is discretized first into octree key cells (default: false).
    *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.*
    */
   public void insertPointCloud(Pointcloud scan, Point3d sensor_origin, double maxrange, boolean lazy_eval, boolean discretize);

   public void insertPointCloud(Pointcloud scan, Point3d sensor_origin, RigidBodyTransform frame_origin)
   {
      insertPointCloud(scan, sensor_origin, -1.0, false, false);
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
   * @param sensor_origin origin of sensor relative to frame origin
   * @param frame_origin origin of reference frame, determines transform to be applied to cloud and sensor origin
   * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
   * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
   *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
   * @param discretize whether the scan is discretized first into octree key cells (default: false).
   *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.*
   */
   public void insertPointCloud(Pointcloud scan, Point3d sensor_origin, RigidBodyTransform frame_origin, double maxrange, boolean lazy_eval,
         boolean discretize);

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
   * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
   * @param lazy_eval whether the tree is left 'dirty' after the update (default: false).
   *   This speeds up the insertion by not updating inner nodes, but you need to call updateInnerOccupancy() when done.
   * @param discretize whether the scan is discretized first into octree key cells (default: false).
   *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.
   */
   public void insertPointCloud(ScanNode scan, double maxrange, boolean lazy_eval, boolean discretize)
   {
      // performs transformation to data and sensor origin first
      Pointcloud cloud = scan.scan;
      RigidBodyTransform frame_origin = new RigidBodyTransform(scan.pose);
      frame_origin.invert();
      Vector3d tempVector = new Vector3d();
      scan.pose.get(tempVector);
      Point3d sensor_origin = new Point3d(tempVector);//frame_origin.inv().transform(scan.pose.trans());
      frame_origin.transform(sensor_origin);
      insertPointCloud(cloud, sensor_origin, frame_origin, maxrange, lazy_eval, discretize);
   }

   public void insertPointCloudRays(Pointcloud scan, Point3d sensor_origin)
   {
      insertPointCloudRays(scan, sensor_origin, -1.0, false);
   }

   /**
    * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
    * This function simply inserts all rays of the point clouds as batch operation.
    * Discretization effects can lead to the deletion of occupied space, it is
    * usually recommended to use insertPointCloud() instead.
    *
    * @param scan Pointcloud (measurement endpoints), in global reference frame
    * @param sensor_origin measurement origin in global reference frame
    * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    */
   public void insertPointCloudRays(Pointcloud scan, Point3d sensor_origin, double maxrange, boolean lazy_eval);

   public NODE setNodeValue(OcTreeKey key, float log_odds_value)
   {
      return setNodeValue(key, log_odds_value, false);
   }

   /**
    * Set log_odds value of voxel to log_odds_value. This only works if key is at the lowest
    * octree level
    *
    * @param key OcTreeKey of the NODE that is to be updated
    * @param log_odds_value value to be set as the log_odds value of the node
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE setNodeValue(OcTreeKey key, float log_odds_value, boolean lazy_eval);

   public NODE setNodeValue(Point3d value, float log_odds_value)
   {
      return setNodeValue(value, log_odds_value, false);
   }

   /**
    * Set log_odds value of voxel to log_odds_value.
    * Looks up the OcTreeKey corresponding to the coordinate and then calls setNodeValue() with it.
    *
    * @param value 3d coordinate of the NODE that is to be updated
    * @param log_odds_value value to be set as the log_odds value of the node
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE setNodeValue(Point3d value, float log_odds_value, boolean lazy_eval);

   public NODE setNodeValue(double x, double y, double z, float log_odds_value)
   {
      return setNodeValue(x, y, z, log_odds_value, false);
   }

   /**
    * Set log_odds value of voxel to log_odds_value.
    * Looks up the OcTreeKey corresponding to the coordinate and then calls setNodeValue() with it.
    *
    * @param x
    * @param y
    * @param z
    * @param log_odds_value value to be set as the log_odds value of the node
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE setNodeValue(double x, double y, double z, float log_odds_value, boolean lazy_eval);

   public NODE updateNode(OcTreeKey key, float log_odds_update)
   {
      return updateNode(key, log_odds_update, false);
   }

   /**
    * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
    * This only works if key is at the lowest octree level
    *
    * @param key OcTreeKey of the NODE that is to be updated
    * @param log_odds_update value to be added (+) to log_odds value of node
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(OcTreeKey key, float log_odds_update, boolean lazy_eval);

   public NODE updateNode(Point3d value, float log_odds_update)
   {
      return updateNode(value, log_odds_update, false);
   }

   /**
    * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
    * Looks up the OcTreeKey corresponding to the coordinate and then calls updateNode() with it.
    *
    * @param value 3d coordinate of the NODE that is to be updated
    * @param log_odds_update value to be added (+) to log_odds value of node
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(Point3d value, float log_odds_update, boolean lazy_eval);

   public NODE updateNode(double x, double y, double z, float log_odds_update)
   {
      return updateNode(x, y, z, log_odds_update, false);
   }

   /**
    * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
    * Looks up the OcTreeKey corresponding to the coordinate and then calls updateNode() with it.
    *
    * @param x
    * @param y
    * @param z
    * @param log_odds_update value to be added (+) to log_odds value of node
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(double x, double y, double z, float log_odds_update, boolean lazy_eval);

   public NODE updateNode(OcTreeKey key, boolean occupied)
   {
      return updateNode(key, occupied, false);
   }

   /**
    * Integrate occupancy measurement.
    *
    * @param key OcTreeKey of the NODE that is to be updated
    * @param occupied true if the node was measured occupied, else false
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(OcTreeKey key, boolean occupied, boolean lazy_eval);

   public NODE updateNode(Point3d value, boolean occupied)
   {
      return updateNode(value, occupied, false);
   }

   /**
    * Integrate occupancy measurement.
    * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
    *
    * @param value 3d coordinate of the NODE that is to be updated
    * @param occupied true if the node was measured occupied, else false
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(Point3d value, boolean occupied, boolean lazy_eval);

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
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(double x, double y, double z, boolean occupied, boolean lazy_eval);

   /**
    * Creates the maximum likelihood map by calling toMaxLikelihood on all
    * tree nodes, setting their occupancy to the corresponding occupancy thresholds.
    * This enables a very efficient compression if you call prune() afterwards.
    */
   public void toMaxLikelihood();

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
    * @param maxrange maximum range after which the raycast should be aborted
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return success of operation
    */
   public boolean insertRay(Point3d origin, Point3d end, double maxrange, boolean lazy_eval);

   public boolean castRay(Point3d origin, Point3d direction, Point3d end)
   {
      return castRay(origin, direction, end, false, -1.0);
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
   public boolean castRay(Point3d origin, Point3d direction, Point3d end, boolean ignoreUnknownCells, double maxRange);

   public boolean getRayIntersection(Point3d origin, Point3d direction, Point3d center, Point3d intersection)
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
   public boolean getRayIntersection(Point3d origin, Point3d direction, Point3d center, Point3d intersection, double delta);

   public boolean getNormals(Point3d point, List<Point3d> normals)
   {
      return getNormals(point, normals, true);
   }

   /**
      * Performs a step of the marching cubes surface reconstruction algorithm
      * to retreive the normal of the triangles that fall in the cube
      * formed by the voxels located at the vertex of a given voxel.
      *
      * @param[in] voxel for which retreive the normals
      * @param[out] triangles normals
      * @param[in] unknownStatus consider unknown cells as free (false) or occupied (default, true).
      * @return True if the input voxel is known in the occupancy grid, and false if it is unknown.
      */
   public boolean getNormals(Point3d point, List<Point3d> normals, boolean unknownStatus);

   //-- set BBX limit (limits tree updates to this bounding box)

   ///  use or ignore BBX limit (default: ignore)
   public void useBBXLimit(boolean enable)
   {
      use_bbx_limit = enable;
   }

   public boolean bbxSet()
   {
      return use_bbx_limit;
   }

   /// sets the minimum for a query bounding box to use
   public void setBBXMin(Point3d min);

   /// sets the maximum for a query bounding box to use
   public void setBBXMax(Point3d max);

   /// @return the currently set minimum for bounding box queries, if set
   public Point3d getBBXMin()
   {
      return bbx_min;
   }

   /// @return the currently set maximum for bounding box queries, if set
   public Point3d getBBXMax()
   {
      return bbx_max;
   }

   public Point3d getBBXBounds();

   public Point3d getBBXCenter();

   /// @return true if point is in the currently set bounding box
   public boolean inBBX(Point3d p);

   /// @return true if key is in the currently set bounding box
   public boolean inBBX(OcTreeKey key);

   //-- change detection on occupancy:
   /// track or ignore changes while inserting scans (default: ignore)
   public void enableChangeDetection(boolean enable)
   {
      use_change_detection = enable;
   }

   public boolean isChangeDetectionEnabled()
   {
      return use_change_detection;
   }

   /// Reset the set of changed keys. Call this after you obtained all changed nodes.
   public void resetChangeDetection()
   {
      changed_keys.clear();
   }

   /// Number of changes since last reset.
   public int numChangesDetected()
   {
      return changed_keys.size();
   }

   /**
    * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
    * integration at once. Here, occupied nodes have a preference over free
    * ones.
    *
    * @param scan point cloud measurement to be integrated
    * @param origin origin of the sensor for ray casting
    * @param free_cells keys of nodes to be cleared
    * @param occupied_cells keys of nodes to be marked occupied
    * @param maxrange maximum range for raycasting (-1: unlimited)
    */
   public void computeUpdate(Pointcloud scan, Point3d origin, KeySet free_cells, KeySet occupied_cells, double maxrange);

   /**
    * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
    * integration at once. Here, occupied nodes have a preference over free
    * ones. This function first discretizes the scan with the octree grid, which results
    * in fewer raycasts (=speedup) but a slightly different result than computeUpdate().
    *
    * @param scan point cloud measurement to be integrated
    * @param origin origin of the sensor for ray casting
    * @param free_cells keys of nodes to be cleared
    * @param occupied_cells keys of nodes to be marked occupied
    * @param maxrange maximum range for raycasting (-1: unlimited)
    */
   public void computeDiscreteUpdate(Pointcloud scan, Point3d origin, KeySet free_cells, KeySet occupied_cells, double maxrange);

   /**
    * Updates the occupancy of all inner nodes to reflect their children's occupancy.
    * If you performed batch-updates with lazy evaluation enabled, you must call this
    * before any queries to ensure correct multi-resolution behavior.
    **/
   public void updateInnerOccupancy();

   /// integrate a "hit" measurement according to the tree's sensor model
   public void integrateHit(NODE occupancyNode);
   /// integrate a "miss" measurement according to the tree's sensor model
   public void integrateMiss(NODE occupancyNode);
   /// update logodds value of node by adding to the current value.
   public void updateNodeLogOdds(NODE occupancyNode, float& update);

   /// converts the node to the maximum likelihood value according to the tree's parameter for "occupancy"
   public void nodeToMaxLikelihood(NODE occupancyNode);

   protected boolean integrateMissOnRay(Point3d origin, Point3d end)
   {
      return integrateMissOnRay(origin, end, false);
   }
   
   /**
    * Traces a ray from origin to end and updates all voxels on the
    *  way as free.  The volume containing "end" is not updated.
    */
   protected boolean integrateMissOnRay(Point3d origin, Point3d end, boolean lazy_eval);

   // recursive calls ----------------------------

   protected NODE updateNodeRecurs(NODE node, boolean node_just_created, OcTreeKey key,
         int depth, float& log_odds_update)
   {
      return updateNodeRecurs(node, node_just_created, key, depth, log_odds_update, false);
   }

   protected NODE updateNodeRecurs(NODE node, boolean node_just_created, OcTreeKey key,
                          int depth, float& log_odds_update, boolean lazy_eval = false);
   
   protected NODE setNodeValueRecurs(NODE node, boolean node_just_created, OcTreeKey key,
         int depth, float log_odds_value)
   {
      return setNodeValueRecurs(node, node_just_created, key, depth, log_odds_value, false);
   }

   protected NODE setNodeValueRecurs(NODE node, boolean node_just_created, OcTreeKey key,
                          int depth, float log_odds_value, boolean lazy_eval);

   protected void updateInnerOccupancyRecurs(NODE node, int depth);
   
   protected void toMaxLikelihoodRecurs(NODE node, int depth, int max_depth);

}
