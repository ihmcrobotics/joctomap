package us.ihmc.jOctoMap.tools;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class JOctoMapGeometryTools
{
   /**
    * Computes the complete minimum rotation from {@code zUp = (0, 0, 1)} to the given
    * {@code vector} and packs it into an {@link AxisAngle4d}. The rotation axis if perpendicular to
    * both vectors. The rotation angle is computed as the angle from the {@code zUp} to the
    * {@code vector}: <br>
    * {@code rotationAngle = zUp.angle(vector)}. </br>
    * Note: the vector does not need to be unit length.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vector is aligned with {@code zUp}: the rotation angle is equal to {@code 0.0} and the
    * rotation axis is set to: (1, 0, 0).
    * <li>the vector is collinear pointing opposite direction of {@code zUp}: the rotation angle is
    * equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of the given normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more collinear.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param vector the 3D vector that is rotated with respect to {@code zUp}. Not modified.
    * @return the minimum rotation from {@code zUp} to the given {@code vector}.
    */
   public static AxisAngle4d getAxisAngleFromZUpToVector(Vector3d vector)
   {
      AxisAngle4d axisAngle = new AxisAngle4d();
      getAxisAngleFromZUpToVector(vector, axisAngle);
      return axisAngle;
   }

   /**
    * Computes the complete minimum rotation from {@code zUp = (0, 0, 1)} to the given
    * {@code vector} and packs it into an {@link AxisAngle4d}. The rotation axis if perpendicular to
    * both vectors. The rotation angle is computed as the angle from the {@code zUp} to the
    * {@code vector}: <br>
    * {@code rotationAngle = zUp.angle(vector)}. </br>
    * Note: the vector does not need to be unit length.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vector is aligned with {@code zUp}: the rotation angle is equal to {@code 0.0} and the
    * rotation axis is set to: (1, 0, 0).
    * <li>the vector is collinear pointing opposite direction of {@code zUp}: the rotation angle is
    * equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of the given normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more collinear.
    * </p>
    * 
    * @param vector the vector that is rotated with respect to {@code zUp}. Not modified.
    * @param rotationToPack the minimum rotation from {@code zUp} to the given {@code vector}.
    *           Modified.
    */
   public static void getAxisAngleFromZUpToVector(Vector3d vector, AxisAngle4d rotationToPack)
   {
      getAxisAngleFromFirstToSecondVector(0.0, 0.0, 1.0, vector.getX(), vector.getY(), vector.getZ(), rotationToPack);
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector}
    * and packs it into an {@link AxisAngle4d}. The rotation axis if perpendicular to both vectors.
    * The rotation angle is computed as the angle from the {@code firstVector} to the
    * {@code secondVector}: <br>
    * {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis
    * is set to: (1, 0, 0).
    * <li>the vectors are collinear pointing opposite directions: the rotation angle is equal to
    * {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more collinear.
    * </p>
    * 
    * @param firstVector the first vector. Not modified.
    * @param secondVector the second vector that is rotated with respect to the first vector. Not
    *           modified.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the
    *           {@code secondVector}. Modified.
    */
   public static void getAxisAngleFromFirstToSecondVector(Vector3d firstVector, Vector3d secondVector, AxisAngle4d rotationToPack)
   {
      getAxisAngleFromFirstToSecondVector(firstVector.getX(), firstVector.getY(), firstVector.getZ(), secondVector.getX(), secondVector.getY(),
                                          secondVector.getZ(), rotationToPack);
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector}
    * and packs it into an {@link AxisAngle4d}. The rotation axis if perpendicular to both vectors.
    * The rotation angle is computed as the angle from the {@code firstVector} to the
    * {@code secondVector}: <br>
    * {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis
    * is set to: (1, 0, 0).
    * <li>the vectors are collinear pointing opposite directions: the rotation angle is equal to
    * {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more collinear.
    * </p>
    * 
    * @param firstVectorX x-component of the first vector.
    * @param firstVectorY y-component of the first vector.
    * @param firstVectorZ z-component of the first vector.
    * @param secondVectorX x-component of the second vector that is rotated with respect to the
    *           first vector.
    * @param secondVectorY y-component of the second vector that is rotated with respect to the
    *           first vector.
    * @param secondVectorZ z-component of the second vector that is rotated with respect to the
    *           first vector.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the
    *           {@code secondVector}. Modified.
    */
   public static void getAxisAngleFromFirstToSecondVector(double firstVectorX, double firstVectorY, double firstVectorZ, double secondVectorX,
                                                          double secondVectorY, double secondVectorZ, AxisAngle4d rotationToPack)
   {
      double rotationAxisX = firstVectorY * secondVectorZ - firstVectorZ * secondVectorY;
      double rotationAxisY = firstVectorZ * secondVectorX - firstVectorX * secondVectorZ;
      double rotationAxisZ = firstVectorX * secondVectorY - firstVectorY * secondVectorX;
      double rotationAxisLength = Math.sqrt(rotationAxisX * rotationAxisX + rotationAxisY * rotationAxisY + rotationAxisZ * rotationAxisZ);

      boolean normalsAreParallel = rotationAxisLength < 1e-7;

      double dot;
      dot = secondVectorX * firstVectorX;
      dot += secondVectorY * firstVectorY;
      dot += secondVectorZ * firstVectorZ;

      if (normalsAreParallel)
      {
         double rotationAngle = dot > 0.0 ? 0.0 : Math.PI;
         rotationToPack.set(1.0, 0.0, 0.0, rotationAngle);
         return;
      }

      double rotationAngle = getAngleFromFirstToSecondVector(firstVectorX, firstVectorY, firstVectorZ, secondVectorX, secondVectorY, secondVectorZ);

      rotationAxisX /= rotationAxisLength;
      rotationAxisY /= rotationAxisLength;
      rotationAxisZ /= rotationAxisLength;
      rotationToPack.set(rotationAxisX, rotationAxisY, rotationAxisZ, rotationAngle);
   }

   /**
    * Computes the angle in radians from the first 3D vector to the second 3D vector. The computed
    * angle is in the range [0; <i>pi</i>].
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either vector is below {@code 1.0E-7}, this method fails and returns an
    * angle of {@code 0.0} radian.
    * </ul>
    * </p>
    * 
    * @param firstVectorX x-component of first the vector.
    * @param firstVectorY y-component of first the vector.
    * @param firstVectorZ z-component of first the vector.
    * @param secondVectorX x-component of second the vector.
    * @param secondVectorY y-component of second the vector.
    * @param secondVectorZ z-component of second the vector.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double getAngleFromFirstToSecondVector(double firstVectorX, double firstVectorY, double firstVectorZ, double secondVectorX,
                                                        double secondVectorY, double secondVectorZ)
   {
      double firstVectorLength = Math.sqrt(firstVectorX * firstVectorX + firstVectorY * firstVectorY + firstVectorZ * firstVectorZ);

      if (firstVectorLength < 1e-7)
         return 0.0;

      double secondVectorLength = Math.sqrt(secondVectorX * secondVectorX + secondVectorY * secondVectorY + secondVectorZ * secondVectorZ);

      if (secondVectorLength < 1e-7)
         return 0.0;

      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY + firstVectorZ * secondVectorZ;
      dotProduct /= firstVectorLength * secondVectorLength;

      if (dotProduct < -1.0)
         dotProduct = -1.0;
      else if (dotProduct > 1.0)
         dotProduct = 1.0;

      return Math.acos(dotProduct);
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
    * 
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
