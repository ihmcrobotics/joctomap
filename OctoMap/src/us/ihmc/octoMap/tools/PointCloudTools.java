package us.ihmc.octoMap.tools;

import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.octoMap.pointCloud.PointCloud;

public abstract class PointCloudTools
{
   public static PointCloud createRandomSample(float[] points, int numberOfSamples)
   {
      PointCloud pointCloud = new PointCloud();
      int numberOfPoints = points.length / 3;

      if (numberOfPoints <= numberOfSamples)
      {
         pointCloud.addAll(points);
      }
      else
      {
         Random random = new Random();
         HashSet<Integer> indices = new HashSet<>(numberOfSamples);

         while (indices.size() < numberOfSamples)
            indices.add(random.nextInt(numberOfPoints));

         for (int index : indices)
         {
            float x = points[3 * index];
            float y = points[3 * index + 1];
            float z = points[3 * index + 2];
            pointCloud.add(x, y, z);
         }
      }
      return pointCloud;
   }

   /**
    * Create a new PointCloud cropped to given bounding box
    */
   public static PointCloud crop(PointCloud input, Point3d lowerBound, Point3d upperBound)
   {
      if (input.isEmpty())
         return new PointCloud(input);

      List<Point3f> pointsInside = input.parallelStream().collect(Collectors.groupingBy(point -> isInsideBounds(point, lowerBound, upperBound))).get(true);
      return new PointCloud(pointsInside);
   }

   /**
    * Removes all the points present in the given sphere.
    */
   public static PointCloud removePointsInsideSphere(PointCloud input, float sphereRadius, Point3f sphereCenter)
   {
      if (input.isEmpty())
         return new PointCloud(input);

      float minimumDistanceSquared = sphereRadius * sphereRadius;
      
      List<Point3f> pointsOutsideSphere = input.parallelStream().collect(Collectors.groupingBy(point -> point.distanceSquared(sphereCenter) > minimumDistanceSquared)).get(true);
      return new PointCloud(pointsOutsideSphere);
   }

   private static boolean isInsideBounds(Point3f point, Point3d lowerBound, Point3d upperBound)
   {
      if (point.getX() < lowerBound.getX() || point.getX() > upperBound.getX())
         return false;
      if (point.getY() < lowerBound.getY() || point.getY() > upperBound.getY())
         return false;
      if (point.getZ() < lowerBound.getZ() || point.getZ() > upperBound.getZ())
         return false;
      return true;
   }
}
