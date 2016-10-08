package us.ihmc.octoMap.testTools;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.octoMap.pointCloud.SweepCollection;

public class OctoMapRandomTools
{
   public static double generateRandomDouble(Random random, double maxAbsolute)
   {
      return generateRandomDouble(random, -maxAbsolute, maxAbsolute);
   }

   public static double generateRandomDouble(Random random, double boundaryOne, double boundaryTwo)
   {
      return boundaryOne + random.nextDouble() * (boundaryTwo - boundaryOne);
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
         pointCloud.add(OctoMapRandomTools.generateRandomPoint3f(random, xRange, yRange, zRange));
      return pointCloud;
   }

   public static SweepCollection generateRandomSweepCollection(Random random, float xRange, float yRange, float zRange, Point3d sensorOrigin, int sweepSize,
         int pointCloudSize)
   {
      SweepCollection sweepCollection = new SweepCollection();

      for (int i = 0; i < sweepSize; i++)
         sweepCollection.addSweep(generateRandomPointCloud(random, xRange, yRange, zRange, pointCloudSize), sensorOrigin);
      return sweepCollection;
   }
}
