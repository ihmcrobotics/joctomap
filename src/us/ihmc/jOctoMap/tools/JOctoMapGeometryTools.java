package us.ihmc.jOctoMap.tools;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class JOctoMapGeometryTools
{
   /**
    * Compute the orthogonal distance from a point to a line (defined by two 3D points).
    * From http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
    * ^^Modified to return distance if line is defined by same point^^
    *          returns
    * @param point query
    * @param lineStart start point of the line.
    * @param lineEnd end point of the line.
    * @return double distance between the point and the line.
    */
   public static double distanceFromPointToLine(Point3d point, Point3d lineStart, Point3d lineEnd)
   {
      if (lineStart.equals(lineEnd))
      {
         double dx = lineStart.getX() - point.getX();
         double dy = lineStart.getY() - point.getY();
         double dz = lineStart.getZ() - point.getZ();
         return Math.sqrt(dx * dx + dy * dy + dz * dz);
      }
      else
      {
         Vector3d startToEnd = new Vector3d(lineEnd);
         startToEnd.sub(lineStart);

         Vector3d crossProduct = new Vector3d(lineStart);
         crossProduct.sub(point);
         crossProduct.cross(startToEnd, crossProduct);

         return crossProduct.length() / startToEnd.length();
      }
   }

   /**
    * Computes the smallest rotation from the given vector to the z-up vector (0, 0, 1).
    * @param normalVector3d the vector to compute the rotation from.
    * @return the resulting rotation.
    */
   public static AxisAngle4d getRotationBasedOnNormal(Vector3d normalVector3d)
   {
      AxisAngle4d rotation = new AxisAngle4d();
      getRotationBasedOnNormal(rotation, normalVector3d);
      return rotation;
   }

   /**
    * Computes the smallest rotation from the given vector to the z-up vector (0, 0, 1).
    * @param rotationToPack the computed rotation.
    * @param normalVector3d the vector to compute the rotation from.
    */
   public static void getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d normalVector3d)
   {
      Vector3d referenceNormal = new Vector3d(0.0, 0.0, 1.0);
      getRotationBasedOnNormal(rotationToPack, normalVector3d, referenceNormal);
   }

   /**
    * Computes the smallest rotation from the reference normal to the rotated normal.
    * @param rotationToPack the computed rotation.
    * @param rotatedNormal the normal that is rotated.
    * @param referenceNormal the reference normal. What should be rotated normal be when there is no rotation applied to it.
    */
   public static void getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d rotatedNormal, Vector3d referenceNormal)
   {
      Vector3d rotationAxis = new Vector3d();

      double inverseLengths = 1.0 / (rotatedNormal.length() * referenceNormal.length());

      rotationAxis.cross(referenceNormal, rotatedNormal);
      double rotationAxisLength = rotationAxis.length();

      double sin = rotationAxisLength * inverseLengths;
      double cos = referenceNormal.dot(rotatedNormal) * inverseLengths;
      double rotationAngle = Math.atan2(sin, cos);

      boolean normalsAreParallel = rotationAxisLength < 1.0e-10;
      if (normalsAreParallel)
      {
         rotationAngle = rotatedNormal.getZ() > 0.0 ? 0.0 : Math.PI;
         rotationAxis.set(1.0, 0.0, 0.0);
      }
      else
      {
         rotationAxis.scale(1.0 / rotationAxisLength);
      }

      rotationToPack.set(rotationAxis, rotationAngle);
   }


   /**
    * Finds the intersections of infinite-length ray with an axis-aligned box.
    * 
    * @param min the min 3D coordinates of the box.
    * @param max the max 3D coordinates of the box.
    * @param rayOrigin point from where the ray starts.
    * @param rayDirection direction the ray is going the rayOrigin.
    * @return the found intersection(s) or null if the ray does not intersect the box.
    */
   public static RayBoxIntersectionResult rayBoxIntersection(Point3d min, Point3d max, Point3d rayOrigin, Vector3d rayDirection)
   {
      return rayBoxIntersection(min, max, rayOrigin, rayDirection, Double.POSITIVE_INFINITY);
   }

   /**
    * Finds the intersections of a finite-length ray with an axis-aligned box.
    * 
    * @param min the min 3D coordinates of the box.
    * @param max the max 3D coordinates of the box.
    * @param rayOrigin point from where the ray starts.
    * @param rayDirection direction the ray is going the rayOrigin.
    * @param maxRayLength intersections beyond the max ray length are ignored.
    * @return the found intersection(s) or null if the ray does not intersect the box.
    */
   public static RayBoxIntersectionResult rayBoxIntersection(Point3d min, Point3d max, Point3d rayOrigin, Vector3d rayDirection, double maxRayLength)
   {
      double lengthSquared = rayDirection.lengthSquared();
      if (lengthSquared < 1.0e-7)
         return null;
      else if (Math.abs(lengthSquared - 1.0) > 1.0e-3)
         rayDirection.scale(1.0 / Math.sqrt(lengthSquared));

      double epsilon = 1.0e-10;
      double tmin = Double.NEGATIVE_INFINITY;
      double tmax = Double.POSITIVE_INFINITY;
      double tymin, tymax, tzmin, tzmax;

      if (Math.abs(rayDirection.getX()) < epsilon)
      {
         if (rayOrigin.getX() > max.getX() - epsilon)
            return null;
         else if (rayOrigin.getX() < min.getX() + epsilon)
            return null;
      }
      else if (rayDirection.getX() > 0.0)
      {
         tmin = (min.getX() - rayOrigin.getX()) / rayDirection.getX();
         tmax = (max.getX() - rayOrigin.getX()) / rayDirection.getX();
      }
      else
      {
         tmin = (max.getX() - rayOrigin.getX()) / rayDirection.getX();
         tmax = (min.getX() - rayOrigin.getX()) / rayDirection.getX();
      }

      if (Math.abs(rayDirection.getY()) < epsilon)
      {
         if (rayOrigin.getY() > max.getY() - epsilon)
            return null;
         else if (rayOrigin.getY() < min.getY() + epsilon)
            return null;
      }
      else 
      {
         if (rayDirection.getY() > 0.0)
         {
            tymin = (min.getY() - rayOrigin.getY()) / rayDirection.getY();
            tymax = (max.getY() - rayOrigin.getY()) / rayDirection.getY();
         }
         else
         {
            tymin = (max.getY() - rayOrigin.getY()) / rayDirection.getY();
            tymax = (min.getY() - rayOrigin.getY()) / rayDirection.getY();
         }

         if ((tmin > tymax) || (tymin > tmax))
            return null;

         if (tymin > tmin)
            tmin = tymin;
         if (tymax < tmax)
            tmax = tymax;
      }

      if (Math.abs(rayDirection.getZ()) < epsilon)
      {
         if (rayOrigin.getZ() > max.getZ() - epsilon)
            return null;
         else if (rayOrigin.getZ() < min.getZ() + epsilon)
            return null;
      }
      else
      {
         if (rayDirection.getZ() >= 0.0)
         {
            tzmin = (min.getZ() - rayOrigin.getZ()) / rayDirection.getZ();
            tzmax = (max.getZ() - rayOrigin.getZ()) / rayDirection.getZ();
         }
         else
         {
            tzmin = (max.getZ() - rayOrigin.getZ()) / rayDirection.getZ();
            tzmax = (min.getZ() - rayOrigin.getZ()) / rayDirection.getZ();
         }
         
         if ((tmin > tzmax) || (tzmin > tmax))
            return null;
         
         if (tzmin > tmin)
            tmin = tzmin;
         if (tzmax < tmax)
            tmax = tzmax;
      }

      if (tmax < 0.0 || tmin > maxRayLength)
         return null;

      Point3d enteringIntersection;

      if (tmin < 0.0)
      {
         if (tmax > maxRayLength)
            return null;
         else
            enteringIntersection = null;
      }
      else
      {
         enteringIntersection = new Point3d();
         enteringIntersection.scaleAdd(tmin, rayDirection, rayOrigin);
      }

      Point3d exitingIntersection;

      if (tmax > maxRayLength)
      {
         exitingIntersection = null;
      }
      else
      {
         exitingIntersection = new Point3d();         
         exitingIntersection.scaleAdd(tmax, rayDirection, rayOrigin);
      }

      return new RayBoxIntersectionResult(tmin, enteringIntersection, tmax, exitingIntersection);
   }

   public static class RayBoxIntersectionResult
   {
      private final double entryDistanceFromOrigin;
      private final Point3d enteringIntersection;
      private final double exitDistanceFromOrigin;
      private final Point3d exitingIntersection;

      public RayBoxIntersectionResult(double entryDistanceFromOrigin, Point3d enteringIntersection, double exitDistanceFromOrigin, Point3d exitingIntersection)
      {
         this.entryDistanceFromOrigin = entryDistanceFromOrigin;
         this.enteringIntersection = enteringIntersection;
         this.exitDistanceFromOrigin = exitDistanceFromOrigin;
         this.exitingIntersection = exitingIntersection;
      }

      public double getEntryDistanceFromOrigin()
      {
         return entryDistanceFromOrigin;
      }

      public Point3d getEnteringIntersection()
      {
         return enteringIntersection;
      }

      public double getExitDistanceFromOrigin()
      {
         return exitDistanceFromOrigin;
      }

      public Point3d getExitingIntersection()
      {
         return exitingIntersection;
      }

      @Override
      public String toString()
      {
         return "Entry point: " + enteringIntersection + "\nExit point: " + exitingIntersection;
      }
   }

   /**
    * Computes the normal of the plane P defined by the three points (p0, p1, p2).
    * @param p0 a point on the plane P.
    * @param p1 a point on the plane P.
    * @param p2 a point on the plane P.
    * @return the plane normal.
    */
   public static Vector3d computeNormal(Point3d p0, Point3d p1, Point3d p2)
   {
      double v1_x = p1.getX() - p0.getX();
      double v1_y = p1.getY() - p0.getY();
      double v1_z = p1.getZ() - p0.getZ();
   
      double v2_x = p2.getX() - p0.getX();
      double v2_y = p2.getY() - p0.getY();
      double v2_z = p2.getZ() - p0.getZ();
   
      Vector3d normal = new Vector3d();
      normal.setX(v1_y * v2_z - v1_z * v2_y);
      normal.setY(v2_x * v1_z - v2_z * v1_x);
      normal.setZ(v1_x * v2_y - v1_y * v2_x);
      normal.normalize();
      return normal;
   }
}
