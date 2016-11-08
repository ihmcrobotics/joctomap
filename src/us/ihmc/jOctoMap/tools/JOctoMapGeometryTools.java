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

   public static AxisAngle4d getRotationBasedOnNormal(Vector3d normalVector3d)
   {
      AxisAngle4d rotation = new AxisAngle4d();
      getRotationBasedOnNormal(rotation, normalVector3d);
      return rotation;
   }

   public static void getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d normalVector3d)
   {
      Vector3d referenceNormal = new Vector3d(0.0, 0.0, 1.0);
      getRotationBasedOnNormal(rotationToPack, normalVector3d, referenceNormal);
   }

   public static void getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d rotatedNormal, Vector3d referenceNormal)
   {
      Vector3d rotationAxis = new Vector3d(0.0, 0.0, 0.0);

      rotationAxis.cross(referenceNormal, rotatedNormal);
      double rotationAngle = referenceNormal.angle(rotatedNormal);

      boolean normalsAreParallel = rotationAxis.lengthSquared() < 1.0e-7;
      if (normalsAreParallel)
      {
         rotationAngle = rotatedNormal.getZ() > 0.0 ? 0.0 : Math.PI;
         rotationAxis.set(1.0, 0.0, 0.0);
      }

      rotationToPack.set(rotationAxis, rotationAngle);
   }
}
