package us.ihmc.octoMap.tools;

import static us.ihmc.octoMap.tools.OcTreeKeyTools.adjustKeyAtDepth;
import static us.ihmc.octoMap.tools.OcTreeKeyTools.computeCenterOffsetKey;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.robotics.MathTools;

public abstract class OcTreeKeyConversionTools
{
   /**
    * Converts a single coordinate into a discrete addressing key, with boundary checking.
    *
    * @param coordinate 3d coordinate of a point
    * @param key discrete 16 bit addressing key, result
    * @return key if coordinate is within the octree bounds (valid), -1 otherwise
    */
   public static int coordinateToKey(double coordinate, double resolution, int treeDepth)
   {
      return coordinateToKey(coordinate, treeDepth, resolution, treeDepth);
   }

   /**
    * Converts a single coordinate into a discrete addressing key, with boundary checking.
    *
    * @param coordinate 3d coordinate of a point
    * @param depth level of the key from the top
    * @param key discrete 16 bit addressing key, result
    * @return key if coordinate is within the octree bounds (valid), -1 otherwise
    */
   public static int coordinateToKey(double coordinate, int depth, double resolution, int treeDepth)
   {
      MathTools.checkIfLessOrEqual(depth, treeDepth);

      int centerOffsetKey = computeCenterOffsetKey(treeDepth);
      // scale to resolution and shift center
      int scaledCoord = (int) Math.floor(coordinate / resolution) + centerOffsetKey;
      if (scaledCoord >= 0 && scaledCoord < 2 * centerOffsetKey)
         return adjustKeyAtDepth(scaledCoord, depth, treeDepth);
      else
         return -1;
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey coordinateToKey(Point3f coord, double resolution, int treeDepth)
   {
      return convertCartesianCoordinateToKey(coord.x, coord.y, coord.z, treeDepth, resolution, treeDepth);
   }
   
   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey coordinateToKey(Point3d coord, double resolution, int treeDepth)
   {
      return convertCartesianCoordinateToKey(coord.x, coord.y, coord.z, treeDepth, resolution, treeDepth);
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @param depth level of the key from the top
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey coordinateToKey(Point3d coord, int depth, double resolution, int treeDepth)
   {
      return convertCartesianCoordinateToKey(coord.x, coord.y, coord.z, depth, resolution, treeDepth);
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @param depth level of the key from the top
    * @param keyToPack
    * @return true if point is within the octree (valid), false otherwise
    */
   public static boolean coordinateToKey(Point3d coord, double resolution, int treeDepth, OcTreeKey keyToPack)
   {
      OcTreeKey key = coordinateToKey(coord, treeDepth, resolution, treeDepth);
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
   public static OcTreeKey coordinateToKey(double x, double y, double z, double resolution, int treeDepth)
   {
      return convertCartesianCoordinateToKey(x, y, z, treeDepth, resolution, treeDepth);
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
   public static OcTreeKey convertCartesianCoordinateToKey(double x, double y, double z, int depth, double resolution, int treeDepth)
   {
      int k0 = coordinateToKey(x, depth, resolution, treeDepth);
      if (k0 == -1)
         return null;
      int k1 = coordinateToKey(y, depth, resolution, treeDepth);
      if (k1 == -1)
         return null;
      int k2 = coordinateToKey(z, depth, resolution, treeDepth);
      if (k2 == -1)
         return null;

      return new OcTreeKey(k0, k1, k2);
   }

   /** converts from a discrete key at a given depth into a coordinate corresponding to the key's center */
   public static double keyToCoordinate(int key, int depth, double resolution, int treeDepth)
   {
      MathTools.checkIfLessOrEqual(depth, treeDepth);

      // root is centered on 0 = 0.0
      if (depth == 0)
      {
         return 0.0;
      }
      else if (depth == treeDepth)
      {
         return keyToCoordinate(key, resolution, treeDepth);
      }
      else
      {
         double nodeSize = computeNodeSize(depth, resolution, treeDepth);
         int centerOffsetKey = computeCenterOffsetKey(treeDepth);
         int keyDivider = 1 << treeDepth - depth;
         return (Math.floor((double) (key - centerOffsetKey) / (double) keyDivider) + 0.5) * nodeSize;
//         return keyToCoordinate(adjustKeyAtDepth(key, depth, maxDepth), nodeSize, depth);
      }
   }

   /** converts from a discrete key at the lowest tree level into a coordinate corresponding to the key's center */
   public static double keyToCoordinate(int key, double resolution, int treeDepth)
   {
      int centerOffsetKey = computeCenterOffsetKey(treeDepth);
      return (key - centerOffsetKey + 0.5) * resolution;
   }

   /** converts from an addressing key at the lowest tree level into a coordinate corresponding to the key's center */
   public static Point3d keyToCoordinate(OcTreeKey key, double resolution, int treeDepth)
   {
      return keyToCoordinate(key, treeDepth, resolution, treeDepth);
   }

   /** converts from an addressing key at a given depth into a coordinate corresponding to the key's center */
   public static Point3d keyToCoordinate(OcTreeKey key, int depth, double resolution, int treeDepth)
   {
      double x = keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      double y = keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      double z = keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);
      return new Point3d(x, y, z);
   }

   public static void keyToCoordinate(OcTreeKey key, Point3d coordinateToPack, double resolution, int treeDepth)
   {
      keyToCoordinate(key, treeDepth, coordinateToPack, resolution, treeDepth);
   }

   /** converts from an addressing key at a given depth into a coordinate corresponding to the key's center */
   public static void keyToCoordinate(OcTreeKey key, int depth, Point3d coordinateToPack, double resolution, int treeDepth)
   {
      coordinateToPack.x = keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      coordinateToPack.y = keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      coordinateToPack.z = keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);
   }

   public static double computeNodeSize(int depth, double resolution, int treeDepth)
   {
      MathTools.checkIfLessOrEqual(depth, treeDepth);
      return (1 << treeDepth - depth) * resolution;
   }
}
