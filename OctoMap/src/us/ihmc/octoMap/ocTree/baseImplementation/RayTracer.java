package us.ihmc.octoMap.ocTree.baseImplementation;

import static us.ihmc.octoMap.tools.OcTreeKeyConversionTools.coordinateToKey;
import static us.ihmc.octoMap.tools.OcTreeKeyConversionTools.keyToCoordinate;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.KeyRay;
import us.ihmc.octoMap.key.KeyRayReadOnly;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.key.OcTreeKeySet;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.octoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.octoMap.tools.OcTreeKeyTools;
import us.ihmc.octoMap.tools.OcTreeSearchTools;

public class RayTracer<NODE extends AbstractOcTreeNode<NODE>>
{
   public static final int maxRaySize = 100000;

   private final OcTreeKey keyOrigin = new OcTreeKey();
   private final OcTreeKey keyEnd = new OcTreeKey();
   private final OcTreeKey currentKey = new OcTreeKey();
   private final OcTreeKeySet unfilteredFreeCells = new OcTreeKeySet();

   private final KeyRay ray = new KeyRay();

   private final Vector3d direction = new Vector3d();
   private final double[] directionArray = new double[3];
   private final double[] originArray = new double[3];
   private final int[] step = new int[3];
   private final double[] tMax = new double[3];
   private final double[] tDelta = new double[3];

   public RayTracer()
   {
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
    * @param maxRange maximum range for raycasting (-1: unlimited)
    */
   public void computeDiscreteUpdate(PointCloud scan, Point3d origin, OcTreeKeySet freeCells, OcTreeKeySet occupiedCells, OcTreeBoundingBox boundingBox,
         double minRange, double maxRange, double resolution, int treeDepth)
   {
      PointCloud discretePC = new PointCloud();
      OcTreeKeySet endpoints = new OcTreeKeySet();

      for (int i = 0; i < scan.size(); ++i)
      {
         OcTreeKey key = OcTreeKeyConversionTools.coordinateToKey(scan.getPoint(i), resolution, treeDepth);
         if (endpoints.add(key)) // insertion took place => key was not in set
            discretePC.add(OcTreeKeyConversionTools.keyToCoordinate(key, resolution, treeDepth));
      }

      computeUpdate(discretePC, origin, freeCells, occupiedCells, boundingBox, minRange, maxRange, resolution, treeDepth);
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
    * @param maxRange maximum range for raycasting (-1: unlimited)
    */
   public void computeUpdate(PointCloud scan, Point3d origin, OcTreeKeySet freeCells, OcTreeKeySet occupiedCells, OcTreeBoundingBox boundingBox,
         double minRange, double maxRange, double resolution, int treeDepth)
   {
      unfilteredFreeCells.clear();
      OcTreeKey key = new OcTreeKey();
      Vector3d direction = new Vector3d();
      Point3d point = new Point3d();

      for (int i = 0; i < scan.size(); ++i)
      {
         point.set(scan.getPoint(i));
         direction.sub(point, origin);
         double length = direction.length();

         if (minRange >= 0.0 && length < minRange)
            continue;

         if (boundingBox == null)
         { // no BBX specified
            if (maxRange < 0.0 || length <= maxRange)
            { // is not maxrange meas.
                 // free cells
               if (computeRayKeys(origin, point, resolution, treeDepth) != null)
               {
                  unfilteredFreeCells.addAll(ray);
               }
               // occupied endpoint
               if (OcTreeKeyConversionTools.coordinateToKey(point, resolution, treeDepth, key))
                  occupiedCells.add(key);
            }
            else
            { // user set a maxrange and length is above
               Point3d newEnd = new Point3d();
               newEnd.scaleAdd(maxRange / length, direction, origin);
               if (computeRayKeys(origin, point, resolution, treeDepth) != null)
                  unfilteredFreeCells.addAll(ray);
            } // end if maxrange
         }
         else
         { // BBX was set
              // endpoint in bbx and not maxrange?
            if (boundingBox.isInBoundingBox(point) && (maxRange < 0.0 || length <= maxRange))
            {
               // occupied endpoint
               if (OcTreeKeyConversionTools.coordinateToKey(point, resolution, treeDepth, key))
                  occupiedCells.add(key);

               // update freespace, break as soon as bbx limit is reached
               if (computeRayKeys(origin, point, resolution, treeDepth) != null)
               {
                  for (int j = ray.size() - 1; j >= 0; j--)
                  {
                     OcTreeKeyReadOnly currentKey = ray.get(j);
                     if (boundingBox.isInBoundingBox(currentKey))
                        unfilteredFreeCells.add(currentKey);
                     else
                        break;
                  }
               } // end if compute ray
            } // end if in BBX and not maxrange
         } // end bbx case

      } // end for all points, end of parallel OMP loop

      // prefer occupied cells over free ones (and make sets disjunct)
      for (int i = 0; i < unfilteredFreeCells.size(); i++)
      {
         OcTreeKeyReadOnly possibleFreeCell = unfilteredFreeCells.unsafeGet(i);
         if (!occupiedCells.contains(possibleFreeCell))
            freeCells.add(possibleFreeCell);
      }
   }

   /**
   * Traces a ray from origin to end (excluding), returning an
   * OcTreeKey of all nodes traversed by the beam. You still need to check
   * if a node at that coordinate exists (e.g. with search()).
   *
   * @param origin start coordinate of ray
   * @param end end coordinate of ray
   * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
   * @return Success of operation. Returning false usually means that one of the coordinates is out of the OcTree's range
   */
   public KeyRayReadOnly computeRayKeys(Point3d origin, Point3d end, double resolution, int treeDepth)
   {
      // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
      // basically: DDA in 3D

      ray.clear();

      boolean foundKeyOrigin = coordinateToKey(origin, resolution, treeDepth, keyOrigin);
      boolean foundKeyEnd = coordinateToKey(end, resolution, treeDepth, keyEnd);

      if (!foundKeyOrigin || !foundKeyEnd)
      {
         System.err.println(AbstractOcTreeBase.class.getSimpleName() + " coordinates ( " + origin + " -> " + end + ") out of bounds in computeRayKeys");
         return null;
      }

      if (keyOrigin.equals(keyEnd))
         return ray; // same tree cell, we're done.

      ray.add(keyOrigin);

      // Initialization phase -------------------------------------------------------

      direction.sub(end, origin);
      double length = direction.length();
      direction.scale(1.0 / length);

      direction.get(directionArray);
      origin.get(originArray);

      currentKey.set(keyOrigin);

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
            double voxelBorder = keyToCoordinate(currentKey.getKey(i), resolution, treeDepth);
            voxelBorder += (float) (step[i] * resolution * 0.5);

            tMax[i] = (voxelBorder - originArray[i]) / directionArray[i];
            tDelta[i] = resolution / Math.abs(directionArray[i]);
         }
         else
         {
            tMax[i] = Double.POSITIVE_INFINITY;
            tDelta[i] = Double.POSITIVE_INFINITY;
         }
      }

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

         // advance in direction "dim"
         currentKey.addKey(dim, step[dim]);
         tMax[dim] += tDelta[dim];

         // reached endpoint, key equv?
         if (currentKey.equals(keyEnd))
         {
            done = true;
            break;
         }
         else
         {
            // reached endpoint world coords?
            // dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
            double distanceFromOrigin;// = Math.min(Math.min(tMax[0], tMax[1]), tMax[2]);

            if (tMax[0] < tMax[1])
            {
               if (tMax[0] < tMax[2])
                  distanceFromOrigin = tMax[0];
               else
                  distanceFromOrigin = tMax[2];
            }
            else
            {
               if (tMax[1] < tMax[2])
                  distanceFromOrigin = tMax[1];
               else
                  distanceFromOrigin = tMax[2];
            }
            // if this is longer than the expected ray length, we should have already hit the voxel containing the end point with the code above (key_end).
            // However, we did not hit it due to accumulating discretization errors, so this is the point here to stop the ray as we would never reach the voxel key_end
            if (distanceFromOrigin > length)
            {
               done = true;
               break;
            }
            else
            { // continue to add freespace cells
               ray.add(currentKey);
            }
         }

         if (ray.size() >= maxRaySize - 1)
            break;

      } // end while

      return ray;
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
    * @param[out] endToPack returns the center of the last cell on the ray. If the function returns true, it is occupied.
    * @param[in] ignoreUnknownCells whether unknown cells are ignored (= treated as free). If false (default), the raycast aborts when an unknown cell is hit and returns false.
    * @param[in] maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
    * @return true if an occupied cell was hit, false if the maximum range or octree bounds are reached, or if an unknown node was hit.
    */
   public boolean castRay(NODE root, Point3d origin, Vector3d direction, Point3d endToPack, boolean ignoreUnknownCells, double maxRange, CollidableRule<NODE> collidableRule, double resolution, int treeDepth)
   {
      /// ----------  see OcTreeBase::computeRayKeys  -----------
   
      // Initialization phase -------------------------------------------------------
      OcTreeKey currentKey = OcTreeKeyConversionTools.coordinateToKey(origin, resolution, treeDepth);
      if (currentKey == null)
      {
         System.err.println(AbstractOccupancyOcTree.class.getSimpleName() + " (in castRay): Coordinates out of bounds during ray casting");
         return false;
      }
   
      NODE startingNode = OcTreeSearchTools.search(root, currentKey, treeDepth);
      if (startingNode != null)
      {
         if (collidableRule.isCollidable(startingNode)) // isNodeOccupied(startingNode))
         {
            // Occupied node found at origin 
            // (need to convert from key, since origin does not need to be a voxel center)
            OcTreeKeyConversionTools.keyToCoordinate(currentKey, endToPack, resolution, treeDepth);
            return true;
         }
      }
      else if (!ignoreUnknownCells)
      {
         OcTreeKeyConversionTools.keyToCoordinate(currentKey, endToPack, resolution, treeDepth);
         return false;
      }
   
      direction = new Vector3d(direction);
      direction.normalize();
      boolean maxRangeSet = maxRange > 0.0;
   
      double[] originArray = new double[3];
      origin.get(originArray);
      double[] endArray = new double[3];
      endToPack.get(endArray);
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
            double voxelBorder = OcTreeKeyConversionTools.keyToCoordinate(currentKey.getKey(i), resolution, treeDepth);
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
         System.err.println(AbstractOccupancyOcTree.class.getSimpleName() + " (in castRay): Raycasting in direction (0,0,0) is not possible!");
         return false;
      }
      int keyMaxValue = OcTreeKeyTools.computeMaximumKey(treeDepth);
   
      // for speedup:
      double maxRangeSquared = maxRange * maxRange;
   
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
         if (step[dim] < 0 && currentKey.getKey(dim) == 0 || step[dim] > 0 && currentKey.getKey(dim) == keyMaxValue)
         {
            System.err.println(AbstractOccupancyOcTree.class.getSimpleName() + " (in castRay): Coordinate hit bounds in dim " + dim + ", aborting raycast");
            // return border point nevertheless:
            OcTreeKeyConversionTools.keyToCoordinate(currentKey, endToPack, resolution, treeDepth);
            return false;
         }
   
         // advance in direction "dim"
         currentKey.addKey(dim, step[dim]);
         tMax[dim] += tDelta[dim];
   
         // generate world coords from key
         OcTreeKeyConversionTools.keyToCoordinate(currentKey, endToPack, resolution, treeDepth);
   
         // check for maxrange:
         if (maxRangeSet)
         {
            double distanceFromOriginSquared = 0.0;
            for (int j = 0; j < 3; j++)
            {
               distanceFromOriginSquared += (endArray[j] - originArray[j]) * (endArray[j] - originArray[j]);
            }
            if (distanceFromOriginSquared > maxRangeSquared)
               return false;
   
         }
   
         NODE currentNode = OcTreeSearchTools.search(root, currentKey, treeDepth);
         if (currentNode != null)
         {
            if (collidableRule.isCollidable(currentNode)) // isNodeOccupied(currentNode))
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
}
