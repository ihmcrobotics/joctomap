package us.ihmc.jOctoMap.tools;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

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

   public static Point2d generateRandomPoint2d(Random random, double maxAbsoluteX, double maxAbsoluteY)
   {
      double x = generateRandomDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = generateRandomDouble(random, -maxAbsoluteY, maxAbsoluteY);

      return new Point2d(x, y);
   }

   public static Point3d generateRandomPoint3d(Random random, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      double x = generateRandomDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = generateRandomDouble(random, -maxAbsoluteY, maxAbsoluteY);
      double z = generateRandomDouble(random, -maxAbsoluteZ, maxAbsoluteZ);

      return new Point3d(x, y, z);
   }

   public static float generateRandomFloat(Random random, float maxAbsolute)
   {
      return generateRandomFloat(random, -maxAbsolute, maxAbsolute);
   }

   public static float generateRandomFloat(Random random, float boundaryOne, float boundaryTwo)
   {
      return boundaryOne + random.nextFloat() * (boundaryTwo - boundaryOne);
   }

   public static Point3f generateRandomPoint3f(Random random, float maxAbsoluteX, float maxAbsoluteY, float maxAbsoluteZ)
   {
      float x = generateRandomFloat(random, -maxAbsoluteX, maxAbsoluteX);
      float y = generateRandomFloat(random, -maxAbsoluteY, maxAbsoluteY);
      float z = generateRandomFloat(random, -maxAbsoluteZ, maxAbsoluteZ);

      return new Point3f(x, y, z);
   }

   public static PointCloud generateRandomPointCloud(Random random, float xRange, float yRange, float zRange, int pointCloudSize)
   {
      PointCloud pointCloud = new PointCloud();
      for (int i = 0; i < pointCloudSize; i++)
         pointCloud.add(JOctoMapRandomTools.generateRandomPoint3f(random, xRange, yRange, zRange));
      return pointCloud;
   }

   public static ScanCollection generateRandomSweepCollection(Random random, float xRange, float yRange, float zRange, Point3d sensorOrigin, int sweepSize,
         int pointCloudSize)
   {
      ScanCollection sweepCollection = new ScanCollection();

      for (int i = 0; i < sweepSize; i++)
         sweepCollection.addScan(generateRandomPointCloud(random, xRange, yRange, zRange, pointCloudSize), sensorOrigin);
      return sweepCollection;
   }

   public static ScanCollection createSingleSweepInPlane(Random random, double sensorDistanceFromPlane, Point3d center, Vector3d normal, double length, double width, int numberOfPoints)
   {
      ScanCollection sweepCollection = new ScanCollection();
      normal.normalize();
   
      Matrix3d orientation = new Matrix3d();
      orientation.set(JOctoMapGeometryTools.getRotationBasedOnNormal(normal));
   
      PointCloud pointCloud = new PointCloud();
   
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3d pointInPlane = generateRandomPoint3d(random, length, width, 0.0);
         Point3d pointInWorld = new Point3d();
         orientation.transform(pointInPlane, pointInWorld);
         pointInWorld.add(center);
         pointCloud.add(pointInWorld);
      }
   
      Point3d sensorOrigin = new Point3d();
      sensorOrigin.scaleAdd(sensorDistanceFromPlane, normal, center);
   
      sweepCollection.addScan(pointCloud, sensorOrigin);
      
      return sweepCollection;
   }
}
