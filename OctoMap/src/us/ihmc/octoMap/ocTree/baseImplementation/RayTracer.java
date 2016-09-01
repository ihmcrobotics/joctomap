package us.ihmc.octoMap.ocTree.baseImplementation;

import static us.ihmc.octoMap.tools.OcTreeKeyConversionTools.coordinateToKey;
import static us.ihmc.octoMap.tools.OcTreeKeyConversionTools.keyToCoordinate;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.KeyRay;
import us.ihmc.octoMap.key.KeyRayReadOnly;
import us.ihmc.octoMap.key.OcTreeKey;

public class RayTracer
{
   public static final int maxRaySize = 100000;

   private final OcTreeKey keyOrigin = new OcTreeKey();
   private final OcTreeKey keyEnd = new OcTreeKey();
   private final OcTreeKey currentKey = new OcTreeKey();

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

   // -- raytracing  -----------------------

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
   public boolean computeRayKeys(Point3d origin, Point3d end, double resolution, int treeDepth)
   {
      // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
      // basically: DDA in 3D

      ray.clear();

      boolean foundKeyOrigin = coordinateToKey(origin, resolution, treeDepth, keyOrigin);
      boolean foundKeyEnd = coordinateToKey(end, resolution, treeDepth, keyEnd);

      if (!foundKeyOrigin || !foundKeyEnd)
      {
         System.err.println(AbstractOcTreeBase.class.getSimpleName() + " coordinates ( " + origin + " -> " + end + ") out of bounds in computeRayKeys");
         return false;
      }

      if (keyOrigin.equals(keyEnd))
         return true; // same tree cell, we're done.

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

      return true;
   }

   public KeyRayReadOnly getResult()
   {
      return ray;
   }
}
