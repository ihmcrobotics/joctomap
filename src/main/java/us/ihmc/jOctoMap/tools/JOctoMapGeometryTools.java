package us.ihmc.jOctoMap.tools;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class JOctoMapGeometryTools
{
   /**
    * Finds the intersections of infinite-length ray with an axis-aligned box.
    *
    * @param min          the min 3D coordinates of the box.
    * @param max          the max 3D coordinates of the box.
    * @param rayOrigin    point from where the ray starts.
    * @param rayDirection direction the ray is going the rayOrigin.
    * @return the found intersection(s) or null if the ray does not intersect the box.
    */
   public static RayBoxIntersectionResult rayBoxIntersection(Point3DReadOnly min, Point3DReadOnly max, Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection)
   {
      return rayBoxIntersection(min, max, rayOrigin, rayDirection, Double.POSITIVE_INFINITY);
   }

   /**
    * Finds the intersections of a finite-length ray with an axis-aligned box.
    *
    * @param min          the min 3D coordinates of the box.
    * @param max          the max 3D coordinates of the box.
    * @param rayOrigin    point from where the ray starts.
    * @param rayDirection direction the ray is going the rayOrigin.
    * @param maxRayLength intersections beyond the max ray length are ignored.
    * @return the found intersection(s) or null if the ray does not intersect the box.
    */
   public static RayBoxIntersectionResult rayBoxIntersection(Point3DReadOnly min, Point3DReadOnly max, Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection,
                                                             double maxRayLength)
   {
      double lengthSquared = rayDirection.lengthSquared();
      if (lengthSquared < 1.0e-7)
         return null;
      else if (Math.abs(lengthSquared - 1.0) > 1.0e-3)
      {
         Vector3D vector3d = new Vector3D(rayDirection);
         vector3d.scale(1.0 / Math.sqrt(lengthSquared));
         rayDirection = vector3d;
      }

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

      Point3D enteringIntersection;

      if (tmin < 0.0)
      {
         if (tmax > maxRayLength)
            return null;
         else
            enteringIntersection = null;
      }
      else
      {
         enteringIntersection = new Point3D();
         enteringIntersection.scaleAdd(tmin, rayDirection, rayOrigin);
      }

      Point3D exitingIntersection;

      if (tmax > maxRayLength)
      {
         exitingIntersection = null;
      }
      else
      {
         exitingIntersection = new Point3D();
         exitingIntersection.scaleAdd(tmax, rayDirection, rayOrigin);
      }

      return new RayBoxIntersectionResult(tmin, enteringIntersection, tmax, exitingIntersection);
   }

   public static class RayBoxIntersectionResult
   {
      private final double entryDistanceFromOrigin;
      private final Point3D enteringIntersection;
      private final double exitDistanceFromOrigin;
      private final Point3D exitingIntersection;

      public RayBoxIntersectionResult(double entryDistanceFromOrigin, Point3D enteringIntersection, double exitDistanceFromOrigin, Point3D exitingIntersection)
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

      public Point3D getEnteringIntersection()
      {
         return enteringIntersection;
      }

      public double getExitDistanceFromOrigin()
      {
         return exitDistanceFromOrigin;
      }

      public Point3D getExitingIntersection()
      {
         return exitingIntersection;
      }

      @Override
      public String toString()
      {
         return "Entry point: " + enteringIntersection + "\nExit point: " + exitingIntersection;
      }
   }
}
