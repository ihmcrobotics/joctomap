package us.ihmc.jOctoMap.tools;

import static us.ihmc.jOctoMap.tools.OcTreeKeyTools.*;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;

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
      OctoMapTools.checkIfDepthValid(depth, treeDepth);

      int centerOffsetKey = computeCenterOffsetKey(treeDepth);
      // scale to resolution and shift center
      int keyAtLowestLevel = (int) Math.floor(coordinate / resolution) + centerOffsetKey;
      if (keyAtLowestLevel >= 0 && keyAtLowestLevel < 2 * centerOffsetKey)
         return adjustKeyAtDepth(keyAtLowestLevel, depth, treeDepth);
      else
         return -1;
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey coordinateToKey(Tuple3f coord, double resolution, int treeDepth)
   {
      return coordinateToKey(coord.getX(), coord.getY(), coord.getZ(), treeDepth, resolution, treeDepth);
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @return key if point is within the octree (valid), null otherwise
    */
   public static boolean coordinateToKey(Tuple3f coord, double resolution, int treeDepth, OcTreeKey keyToPack)
   {
      return coordinateToKey(coord.getX(), coord.getY(), coord.getZ(), treeDepth, resolution, treeDepth, keyToPack);
   }
   
   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey coordinateToKey(Tuple3d coord, double resolution, int treeDepth)
   {
      return coordinateToKey(coord.getX(), coord.getY(), coord.getZ(), treeDepth, resolution, treeDepth);
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @param depth level of the key from the top
    * @return key if point is within the octree (valid), null otherwise
    */
   public static OcTreeKey coordinateToKey(Tuple3d coord, int depth, double resolution, int treeDepth)
   {
      return coordinateToKey(coord.getX(), coord.getY(), coord.getZ(), depth, resolution, treeDepth);
   }

   public static boolean coordinateToKey(Tuple3d coord, int depth, double resolution, int treeDepth, OcTreeKey keyToPack)
   {
      return coordinateToKey(coord.getX(), coord.getY(), coord.getZ(), depth, resolution, treeDepth, keyToPack);
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @param depth level of the key from the top
    * @param keyToPack
    * @return true if point is within the octree (valid), false otherwise
    */
   public static boolean coordinateToKey(Tuple3d coord, double resolution, int treeDepth, OcTreeKey keyToPack)
   {
      return coordinateToKey(coord, treeDepth, resolution, treeDepth, keyToPack);
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
      return coordinateToKey(x, y, z, treeDepth, resolution, treeDepth);
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
   public static boolean coordinateToKey(double x, double y, double z, int depth, double resolution, int treeDepth, OcTreeKey keyToPack)
   {
      int k0 = coordinateToKey(x, depth, resolution, treeDepth);
      if (k0 == -1)
         return false;
      int k1 = coordinateToKey(y, depth, resolution, treeDepth);
      if (k1 == -1)
         return false;
      int k2 = coordinateToKey(z, depth, resolution, treeDepth);
      if (k2 == -1)
         return false;

      keyToPack.set(k0, k1, k2);
      return true;
   }

   public static OcTreeKey coordinateToKey(double x, double y, double z, int depth, double resolution, int treeDepth)
   {
      OcTreeKey ret = new OcTreeKey();
      if (coordinateToKey(x, y, z, depth, resolution, treeDepth, ret))
         return ret;
      else
         return null;
   }

   /** converts from a discrete key at a given depth into a coordinate corresponding to the key's center */
   public static double keyToCoordinate(int key, int depth, double resolution, int treeDepth)
   {
      OctoMapTools.checkIfDepthValid(depth, treeDepth);
      checkKeyIsValid(key, treeDepth, treeDepth);

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
      }
   }

   /** converts from a discrete key at the lowest tree level into a coordinate corresponding to the key's center */
   public static double keyToCoordinate(int key, double resolution, int treeDepth)
   {
      int centerOffsetKey = computeCenterOffsetKey(treeDepth);
      checkKeyIsValid(key, treeDepth, treeDepth);
      return (key - centerOffsetKey + 0.5) * resolution;
   }

   /** converts from an addressing key at the lowest tree level into a coordinate corresponding to the key's center */
   public static Point3d keyToCoordinate(OcTreeKeyReadOnly key, double resolution, int treeDepth)
   {
      return keyToCoordinate(key, treeDepth, resolution, treeDepth);
   }

   /** converts from an addressing key at a given depth into a coordinate corresponding to the key's center */
   public static Point3d keyToCoordinate(OcTreeKeyReadOnly key, int depth, double resolution, int treeDepth)
   {
      double x = keyToCoordinate(key.getKey(0), depth, resolution, treeDepth);
      double y = keyToCoordinate(key.getKey(1), depth, resolution, treeDepth);
      double z = keyToCoordinate(key.getKey(2), depth, resolution, treeDepth);
      return new Point3d(x, y, z);
   }

   public static void keyToCoordinate(OcTreeKeyReadOnly key, Tuple3d coordinateToPack, double resolution, int treeDepth)
   {
      keyToCoordinate(key, treeDepth, coordinateToPack, resolution, treeDepth);
   }

   /** converts from an addressing key at a given depth into a coordinate corresponding to the key's center */
   public static void keyToCoordinate(OcTreeKeyReadOnly key, int depth, Tuple3d coordinateToPack, double resolution, int treeDepth)
   {
      coordinateToPack.setX(keyToCoordinate(key.getKey(0), depth, resolution, treeDepth));
      coordinateToPack.setY(keyToCoordinate(key.getKey(1), depth, resolution, treeDepth));
      coordinateToPack.setZ(keyToCoordinate(key.getKey(2), depth, resolution, treeDepth));
   }

   public static double computeNodeSize(int depth, double resolution, int treeDepth)
   {
      OctoMapTools.checkIfDepthValid(depth, treeDepth);
      return (1 << treeDepth - depth) * resolution;
   }
}
