package us.ihmc.octoMap.tools;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.robotics.MathTools;

public abstract class OcTreeCoordinateConversionTools
{
   /**
    * Converts a single coordinate into a discrete addressing key, with boundary checking.
    *
    * @param coordinate 3d coordinate of a point
    * @param key discrete 16 bit addressing key, result
    * @return key if coordinate is within the octree bounds (valid), -1 otherwise
    */
   public static int convertCartesianCoordinateToKey(double coordinate, double resolution, int maxDepth)
   {
      return convertCartesianCoordinateToKey(coordinate, maxDepth, resolution, maxDepth);
   }

   /**
    * Converts a single coordinate into a discrete addressing key, with boundary checking.
    *
    * @param coordinate 3d coordinate of a point
    * @param depth level of the key from the top
    * @param key discrete 16 bit addressing key, result
    * @return key if coordinate is within the octree bounds (valid), -1 otherwise
    */
   public static int convertCartesianCoordinateToKey(double coordinate, int depth, double resolution, int maxDepth)
   {
      MathTools.checkIfLessOrEqual(depth, maxDepth);

      int maxValue = 1 << (maxDepth - 1);
      // scale to resolution and shift center for tree_max_val
      int scaledCoord = ((int) Math.floor(coordinate / resolution)) + maxValue;
      if (scaledCoord >= 0 && scaledCoord < 2 * maxValue)
         return OcTreeKeyTools.adjustKeyAtDepth(scaledCoord, depth, maxDepth);
      else
         return -1;
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey convertCartesianCoordinateToKey(Point3d coord, double resolution, int maxDepth)
   {
      return convertCartesianCoordinateToKey(coord.x, coord.y, coord.z, maxDepth, resolution, maxDepth);
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @param depth level of the key from the top
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey convertCartesianCoordinateToKey(Point3d coord, int depth, double resolution, int maxDepth)
   {
      return convertCartesianCoordinateToKey(coord.x, coord.y, coord.z, depth, resolution, maxDepth);
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @param depth level of the key from the top
    * @param keyToPack
    * @return true if point is within the octree (valid), false otherwise
    */
   public static boolean convertCartesianCoordinateToKey(Point3d coord, double resolution, int maxDepth, OcTreeKey keyToPack)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(coord, maxDepth, resolution, maxDepth);
      if (key == null)
         return false;

      keyToPack.set(key);
      return true;
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param x 3d coordinate of a point
    * @param y 3d coordinate of a point
    * @param z 3d coordinate of a point
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey convertCartesianCoordinateToKey(double x, double y, double z, double resolution, int maxDepth)
   {
      return convertCartesianCoordinateToKey(x, y, z, maxDepth, resolution, maxDepth);
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param x 3d coordinate of a point
    * @param y 3d coordinate of a point
    * @param z 3d coordinate of a point
    * @param depth level of the key from the top
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey convertCartesianCoordinateToKey(double x, double y, double z, int depth, double resolution, int maxDepth)
   {
      int k0 = convertCartesianCoordinateToKey(x, depth, resolution, maxDepth);
      if (k0 == -1)
         return null;
      int k1 = convertCartesianCoordinateToKey(y, depth, resolution, maxDepth);
      if (k1 == -1)
         return null;
      int k2 = convertCartesianCoordinateToKey(z, depth, resolution, maxDepth);
      if (k2 == -1)
         return null;

      return new OcTreeKey(k0, k1, k2);
   }

   /// converts from a discrete key at a given depth into a coordinate
   /// corresponding to the key's center
   public static double convertKeyToCartesianCoordinate(int key, int depth, double resolution, int maxDepth)
   {
      MathTools.checkIfLessOrEqual(depth, maxDepth);

      // root is centered on 0 = 0.0
      if (depth == 0)
      {
         return 0.0;
      }
      else if (depth == maxDepth)
      {
         return convertKeyToCartesianCoordinate(key, resolution, maxDepth);
      }
      else
      {
         double nodeSize = computeNodeSize(depth, resolution, maxDepth);
         return (Math.floor(((double) (key) - (double) (1 << (maxDepth - 1))) / (double) (1 << (maxDepth - depth))) + 0.5) * nodeSize;
      }
   }

   /// converts from a discrete key at the lowest tree level into a coordinate
   /// corresponding to the key's center
   public static double convertKeyToCartesianCoordinate(int key, double resolution, int maxDepth)
   {
      return ((double) (key - (1 << (maxDepth - 1))) + 0.5) * resolution;
   }

   /// converts from an addressing key at the lowest tree level into a coordinate
   /// corresponding to the key's center
   public static Point3d convertKeyToCartesianCoordinate(OcTreeKey key, double resolution, int maxDepth)
   {
      return convertKeyToCartesianCoordinate(key, maxDepth, resolution, maxDepth);
   }

   /// converts from an addressing key at a given depth into a coordinate
   /// corresponding to the key's center
   public static Point3d convertKeyToCartesianCoordinate(OcTreeKey key, int depth, double resolution, int maxDepth)
   {
      double x = convertKeyToCartesianCoordinate(key.k[0], depth, resolution, maxDepth);
      double y = convertKeyToCartesianCoordinate(key.k[1], depth, resolution, maxDepth);
      double z = convertKeyToCartesianCoordinate(key.k[2], depth, resolution, maxDepth);
      return new Point3d(x, y, z);
   }

   public static double computeNodeSize(int depth, double resolution, int maxDepth)
   {
      MathTools.checkIfLessOrEqual(depth, maxDepth);
      return (double) (1 << (maxDepth - depth)) * resolution;
   }
}
