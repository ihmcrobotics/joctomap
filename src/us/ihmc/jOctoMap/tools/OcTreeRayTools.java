package us.ihmc.jOctoMap.tools;

import static us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools.coordinateToKey;
import static us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools.keyToCoordinate;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.key.KeyRay;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKeySet;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.rules.interfaces.CollidableRule;
import us.ihmc.jOctoMap.rules.interfaces.RayActionRule;
import us.ihmc.jOctoMap.tools.JOctoMapGeometryTools.RayBoxIntersectionResult;

public abstract class OcTreeRayTools
{
   private static final boolean DEBUG = false;
   public static final int maxRaySize = 100000;

   /**
    * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
    * integration at once. Here, occupied nodes have a preference over free
    * ones. This function first discretizes the scan with the octree grid, which results
    * in fewer raycasts (=speedup) but a slightly different result than computeUpdate().
    * @param origin origin of the sensor for ray casting
    * @param pointCloud point cloud measurement to be integrated
    * @param freeCells keys of nodes to be cleared
    * @param occupiedCells keys of nodes to be marked occupied
    * @param maxRange maximum range for raycasting (-1: unlimited)
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> void computeDiscreteUpdate(Point3DReadOnly origin, PointCloud pointCloud, OcTreeKeySet freeCells, OcTreeKeySet occupiedCells,
         OcTreeBoundingBoxInterface boundingBox, double minRange, double maxRange, double resolution, int treeDepth)
   {
      PointCloud discretePC = new PointCloud();
      OcTreeKeySet endpoints = new OcTreeKeySet();

      for (int i = 0; i < pointCloud.getNumberOfPoints(); ++i)
      {
         OcTreeKey key = OcTreeKeyConversionTools.coordinateToKey(pointCloud.getPoint(i), resolution, treeDepth);
         if (endpoints.add(key)) // insertion took place => key was not in set
            discretePC.add(OcTreeKeyConversionTools.keyToCoordinate(key, resolution, treeDepth));
      }

      computeUpdate(origin, discretePC, freeCells, occupiedCells, boundingBox, minRange, maxRange, resolution, treeDepth);
   }

   /**
    * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
    * integration at once. Here, occupied nodes have a preference over free
    * ones.
    * @param origin origin of the sensor for ray casting
    * @param pointCloud point cloud measurement to be integrated
    * @param freeCells keys of nodes to be cleared
    * @param occupiedCells keys of nodes to be marked occupied
    * @param maxRange maximum range for raycasting (-1: unlimited)
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> void computeUpdate(Point3DReadOnly origin, PointCloud pointCloud, OcTreeKeySet freeCells, OcTreeKeySet occupiedCells, OcTreeBoundingBoxInterface boundingBox,
         double minRange, double maxRange, double resolution, int treeDepth)
   {
      OcTreeKeySet unfilteredFreeCells = new OcTreeKeySet();
      OcTreeKey key = new OcTreeKey();
      Vector3D direction = new Vector3D();
      Point3D point = new Point3D();

      for (int i = 0; i < pointCloud.getNumberOfPoints(); ++i)
      {
         point.set(pointCloud.getPoint(i));
         direction.sub(point, origin);
         double length = direction.length();

         if (minRange >= 0.0 && length < minRange)
            continue;

         if (boundingBox == null)
         { // no BBX specified
            if (maxRange < 0.0 || length <= maxRange)
            { // is not maxrange meas.
                 // free cells
               KeyRay ray = computeRayKeys(origin, point, resolution, treeDepth);
               if (ray != null)
                  unfilteredFreeCells.addAll(ray);
               // occupied endpoint
               if (OcTreeKeyConversionTools.coordinateToKey(point, resolution, treeDepth, key))
                  occupiedCells.add(key);
            }
            else
            { // user set a maxrange and length is above
               Point3D newEnd = new Point3D();
               newEnd.scaleAdd(maxRange / length, direction, origin);
               KeyRay ray = computeRayKeys(origin, newEnd, resolution, treeDepth);
               if (ray != null)
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
               KeyRay ray = computeRayKeys(origin, point, resolution, treeDepth);
               if (ray != null)
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
      for (OcTreeKeyReadOnly possibleFreeCell : unfilteredFreeCells)
      {
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
   public static KeyRay computeRayKeys(Point3DReadOnly origin, Point3DReadOnly end, double resolution, int treeDepth)
   {
      return computeRayKeys(origin, end, null, resolution, treeDepth);
   }

   public static KeyRay computeRayKeys(Point3DReadOnly origin, Point3DReadOnly end, OcTreeBoundingBoxInterface boundingBox, double resolution, int treeDepth)
   {
      KeyRay keyRay = new KeyRay();
      RayActionRule growKeyRayActionRule = (rayOrigin, rayEnd, rayDirection, key) -> keyRay.add(key);
      doActionOnRayKeys(origin, end, boundingBox, growKeyRayActionRule, resolution, treeDepth);
      return keyRay;
   }

   /**
    * Traces a ray from origin to end (excluding), returning an
    * OcTreeKey of all nodes traversed by the beam. You still need to check
    * if a node at that coordinate exists (e.g. with search()).
    */
   public static void doActionOnRayKeys(Point3DReadOnly origin, Point3DReadOnly end, OcTreeBoundingBoxInterface boundingBox, RayActionRule actionRule, double resolution,
         int treeDepth) 
   {
      Vector3D direction = new Vector3D();
      direction.sub(end, origin);
      double length = direction.length();
      direction.scale(1.0 / length);

      if (boundingBox != null)
      {
         RayBoxIntersectionResult rayIntersection = boundingBox.rayIntersection(origin, direction, length);
         if (rayIntersection != null)
         {
            Point3D exitingIntersection = rayIntersection.getExitingIntersection();
            if (exitingIntersection != null)
            {
               end = exitingIntersection;
               length = rayIntersection.getExitDistanceFromOrigin();
            }

            Point3D enteringIntersection = rayIntersection.getEnteringIntersection();
            if (enteringIntersection != null)
            {
               origin = enteringIntersection;
               length -= rayIntersection.getEntryDistanceFromOrigin();
            }
         }
      }

      // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
      // basically: DDA in 3D
      OcTreeKey keyOrigin = coordinateToKey(origin, resolution, treeDepth);
      OcTreeKey keyEnd = coordinateToKey(end, resolution, treeDepth);

      if (keyOrigin == null || keyEnd == null)
      {
         if (DEBUG)
            System.err.println(OcTreeRayTools.class.getSimpleName() + " coordinates ( " + origin + " -> " + end + ") out of bounds in computeRayKeys");
         return;
      }

      if (keyOrigin.equals(keyEnd))
         return; // same tree cell, we're done.

      double[] directionArray = new double[3];
      double[] originArray = new double[3];
      int[] step = new int[3];
      double[] tMax = new double[3];
      double[] tDelta = new double[3];
      OcTreeKey currentKey = new OcTreeKey();

      // Initialization phase -------------------------------------------------------
      actionRule.doAction(origin, end, direction, keyOrigin);

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
               actionRule.doAction(origin, end, direction, currentKey);
            }
         }
      } // end while
   }

   /**
    * Traces a ray from origin to end (excluding), returning an
    * OcTreeKey of all nodes traversed by the beam. You still need to check
    * if a node at that coordinate exists (e.g. with search()).
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> void doActionOnRayKeysUsingRayMarching(NODE root, Point3DReadOnly origin, Point3DReadOnly end, OcTreeBoundingBoxInterface boundingBox, RayActionRule actionRule,
         double resolution, int treeDepth)
   {
      OcTreeKey keyOrigin = coordinateToKey(origin, resolution, treeDepth);
      OcTreeKey keyEnd = coordinateToKey(end, resolution, treeDepth);

      if (keyOrigin == null || keyEnd == null)
      {
         if (DEBUG)
            System.err.println(OcTreeRayTools.class.getSimpleName() + " coordinates ( " + origin + " -> " + end + ") out of bounds in computeRayKeys");
         return;
      }

      if (keyOrigin.equals(keyEnd))
         return; // same tree cell, we're done.

      Vector3D direction = new Vector3D();
      Point3D currentPosition = new Point3D();
      OcTreeKey currentKey = new OcTreeKey();

      direction.sub(end, origin);
      double rayLength = direction.length();
      direction.scale(1.0 / rayLength);
      double currentLength = 0.0;
      currentPosition.set(origin);

      while (true)
      {
         double distanceFromNearestNeighbor = OcTreeNearestNeighborTools.findNearestNeighbor(root, currentPosition, currentKey);

         if (Double.isNaN(distanceFromNearestNeighbor))
            break;

         if (distanceFromNearestNeighbor < 0.1 * resolution)
         {
            actionRule.doAction(origin, end, direction, currentKey);
            
            Point3D nearestNodeCoordinate = keyToCoordinate(currentKey, resolution, treeDepth);
            double minDistance = 0.1 * resolution;
            distanceFromNearestNeighbor = OcTreeNearestNeighborTools.findNearestNeighbor(root, nearestNodeCoordinate, minDistance, currentKey);
         }

         distanceFromNearestNeighbor = Math.max(distanceFromNearestNeighbor, 0.5 * resolution);

         currentPosition.scaleAdd(distanceFromNearestNeighbor, direction, currentPosition);

         currentLength += distanceFromNearestNeighbor;

         if (currentLength >= rayLength)
            break;

         if (boundingBox != null && !boundingBox.isInBoundingBox(currentPosition))
            break;

         OcTreeKeyConversionTools.coordinateToKey(currentPosition, resolution, treeDepth, currentKey);

         if (currentKey.equals(keyEnd))
            break;
      }
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
   public static <NODE extends AbstractOcTreeNode<NODE>> boolean castRay(NODE root, Point3DReadOnly origin, Vector3DReadOnly direction, Point3DBasics endToPack, boolean ignoreUnknownCells, double maxRange,
         CollidableRule<NODE> collidableRule, double resolution, int treeDepth)
   {
      /// ----------  see OcTreeBase::computeRayKeys  -----------

      // Initialization phase -------------------------------------------------------
      OcTreeKey currentKey = OcTreeKeyConversionTools.coordinateToKey(origin, resolution, treeDepth);
      if (currentKey == null)
      {
         if (DEBUG)
            System.err.println(OcTreeRayTools.class.getSimpleName() + " (in castRay): Coordinates out of bounds during ray casting");
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

      direction = new Vector3D(direction);
      ((Vector3DBasics) direction).normalize();
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
         if (DEBUG)
            System.err.println(OcTreeRayTools.class.getSimpleName() + " (in castRay): Raycasting in direction (0,0,0) is not possible!");
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
            if (DEBUG)
               System.err.println(OcTreeRayTools.class.getSimpleName() + " (in castRay): Coordinate hit bounds in dim " + dim + ", aborting raycast");
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

   /**
    * Retrieves the entry point of a ray into a voxel. This is the closest intersection point of the ray
    * originating from origin and a plane of the axis aligned cube.
    * 
    * @param[in] origin Starting point of ray
    * @param[in] direction A vector pointing in the direction of the raycast. Does not need to be normalized.
    * @param[in] center The center of the voxel where the ray terminated. This is the output of castRay.
    * @param[out] intersectionToPack The entry point of the ray into the voxel, on the voxel surface.
    * @param[in] delta A small increment to avoid ambiguity of being exactly on a voxel surface. A positive value will get the point out of the hit voxel, while a negative value will get it inside.
    * @return Whether or not an intesection point has been found. Either, the ray never cross the voxel or the ray is exactly parallel to the only surface it intersect.
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> boolean getRayIntersection(Point3DReadOnly origin, Vector3DReadOnly direction, Point3DReadOnly center, Point3DBasics intersectionToPack, double delta, double resolution)
   {
      // We only need three normals for the six planes
      Vector3D normalX = new Vector3D(1, 0, 0);
      Vector3D normalY = new Vector3D(0, 1, 0);
      Vector3D normalZ = new Vector3D(0, 0, 1);

      // One point on each plane, let them be the center for simplicity
      Vector3D pointXNeg = new Vector3D(center.getX() - resolution / 2.0, center.getY(), center.getZ());
      Vector3D pointXPos = new Vector3D(center.getX() + resolution / 2.0, center.getY(), center.getZ());
      Vector3D pointYNeg = new Vector3D(center.getX(), center.getY() - resolution / 2.0, center.getZ());
      Vector3D pointYPos = new Vector3D(center.getX(), center.getY() + resolution / 2.0, center.getZ());
      Vector3D pointZNeg = new Vector3D(center.getX(), center.getY(), center.getZ() - resolution / 2.0);
      Vector3D pointZPos = new Vector3D(center.getX(), center.getY(), center.getZ() + resolution / 2.0);

      double lineDotNormal = 0.0;
      double d = 0.0;
      double outD = Double.POSITIVE_INFINITY;
      Point3D intersect = new Point3D();
      boolean found = false;

      Vector3D tempVector = new Vector3D();

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

      // Subtract (add) a fraction to ensure no ambiguity on the starting voxel
      // Don't start on a boundary.
      if (found)
         intersectionToPack.scaleAdd(outD + delta, direction, origin);

      return found;
   }
}
