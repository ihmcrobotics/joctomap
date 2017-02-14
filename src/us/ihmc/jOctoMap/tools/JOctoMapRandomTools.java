package us.ihmc.jOctoMap.tools;

import java.util.Random;

import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.tuple2D.Point2D;
import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Point3D32;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;

public abstract class JOctoMapRandomTools
{
   public static double generateRandomDouble(Random random, double maxAbsolute)
   {
      return generateRandomDouble(random, -maxAbsolute, maxAbsolute);
   }

   public static double generateRandomDouble(Random random, double boundaryOne, double boundaryTwo)
   {
      return boundaryOne + random.nextDouble() * (boundaryTwo - boundaryOne);
   }

   public static Point2D generateRandomPoint2D(Random random, double maxAbsoluteX, double maxAbsoluteY)
   {
      double x = generateRandomDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = generateRandomDouble(random, -maxAbsoluteY, maxAbsoluteY);

      return new Point2D(x, y);
   }

   public static Point3D generateRandomPoint3D(Random random, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      double x = generateRandomDouble(random, min.getX(), max.getX());
      double y = generateRandomDouble(random, min.getY(), max.getY());
      double z = generateRandomDouble(random, min.getZ(), max.getZ());

      return new Point3D(x, y, z);
   }

   public static Point3D generateRandomPoint3D(Random random, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      double x = generateRandomDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = generateRandomDouble(random, -maxAbsoluteY, maxAbsoluteY);
      double z = generateRandomDouble(random, -maxAbsoluteZ, maxAbsoluteZ);

      return new Point3D(x, y, z);
   }
   
   public static Vector3D generateRandomVector3D(Random random, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      double x = generateRandomDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = generateRandomDouble(random, -maxAbsoluteY, maxAbsoluteY);
      double z = generateRandomDouble(random, -maxAbsoluteZ, maxAbsoluteZ);
      
      return new Vector3D(x, y, z);
   }

   public static Vector3D generateRandomVector3D(Random random, double magnitude)
   {
      Vector3D ret = generateRandomVector3D(random, 1.0, 1.0, 1.0);
      ret.normalize();
      ret.scale(magnitude);
      return ret;
   }

   public static Vector3D generateRandomOrthogonalVector3D(Random random, Vector3DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      Vector3D v1 = new Vector3D(vectorToBeOrthogonalTo.getY(), - vectorToBeOrthogonalTo.getX(), 0.0);
      Vector3D v2 = new Vector3D(- vectorToBeOrthogonalTo.getZ(), 0.0, vectorToBeOrthogonalTo.getX());

      Vector3D randomPerpendicular = new Vector3D();
      double a = JOctoMapRandomTools.generateRandomDouble(random, 1.0);
      double b = JOctoMapRandomTools.generateRandomDouble(random, 1.0);
      randomPerpendicular.scaleAdd(a, v1, randomPerpendicular);
      randomPerpendicular.scaleAdd(b, v2, randomPerpendicular);

      if (normalize)
         randomPerpendicular.normalize();

      return randomPerpendicular;
   }

   public static float generateRandomFloat(Random random, float maxAbsolute)
   {
      return generateRandomFloat(random, -maxAbsolute, maxAbsolute);
   }

   public static float generateRandomFloat(Random random, float boundaryOne, float boundaryTwo)
   {
      return boundaryOne + random.nextFloat() * (boundaryTwo - boundaryOne);
   }

   public static Point3D32 generateRandomPoint3D32(Random random, float maxAbsoluteX, float maxAbsoluteY, float maxAbsoluteZ)
   {
      float x = generateRandomFloat(random, -maxAbsoluteX, maxAbsoluteX);
      float y = generateRandomFloat(random, -maxAbsoluteY, maxAbsoluteY);
      float z = generateRandomFloat(random, -maxAbsoluteZ, maxAbsoluteZ);

      return new Point3D32(x, y, z);
   }

   public static PointCloud generateRandomPointCloud(Random random, float xRange, float yRange, float zRange, int pointCloudSize)
   {
      PointCloud pointCloud = new PointCloud();
      for (int i = 0; i < pointCloudSize; i++)
         pointCloud.add(JOctoMapRandomTools.generateRandomPoint3D32(random, xRange, yRange, zRange));
      return pointCloud;
   }

   public static ScanCollection generateRandomSweepCollection(Random random, float xRange, float yRange, float zRange, Point3DReadOnly sensorOrigin, int sweepSize,
         int pointCloudSize)
   {
      ScanCollection sweepCollection = new ScanCollection();

      for (int i = 0; i < sweepSize; i++)
         sweepCollection.addScan(generateRandomPointCloud(random, xRange, yRange, zRange, pointCloudSize), sensorOrigin);
      return sweepCollection;
   }

   public static ScanCollection createSingleSweepInPlane(Random random, double sensorDistanceFromPlane, Point3DReadOnly center, Vector3DBasics normal, double length, double width, int numberOfPoints)
   {
      ScanCollection sweepCollection = new ScanCollection();
      normal.normalize();
   
      RotationMatrix orientation = new RotationMatrix(JOctoMapGeometryTools.getAxisAngleFromZUpToVector(normal));
   
      PointCloud pointCloud = new PointCloud();
   
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D pointInPlane = generateRandomPoint3D(random, length, width, 0.0);
         Point3D pointInWorld = new Point3D();
         orientation.transform(pointInPlane, pointInWorld);
         pointInWorld.add(center);
         pointCloud.add(pointInWorld);
      }
   
      Point3D sensorOrigin = new Point3D();
      sensorOrigin.scaleAdd(sensorDistanceFromPlane, normal, center);
   
      sweepCollection.addScan(pointCloud, sensorOrigin);
      
      return sweepCollection;
   }
}
