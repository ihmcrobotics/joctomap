package us.ihmc.octoMap.ocTree.implementations;

import java.util.Scanner;

public class PlanarRegionSegmentationParameters
{
   public static final double DEFAULT_SEARCH_RADIUS = 0.05;
   public static final double DEFAULT_MAX_DISTANCE_FROM_PLANE = 0.05;
   public static final double DEFAULT_MAX_ANGLE_FROM_PLANE = Math.toRadians(10.0);
   public static final double DEFAULT_MIN_NORMAL_QUALITY = 0.005;

   private double searchRadius;
   private double maxDistanceFromPlane;
   private double maxAngleFromPlane;
   private double minNormalQuality;

   public PlanarRegionSegmentationParameters()
   {
      setDefaultParameters();
   }

   public PlanarRegionSegmentationParameters(PlanarRegionSegmentationParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      searchRadius = DEFAULT_SEARCH_RADIUS;
      maxDistanceFromPlane = DEFAULT_MAX_DISTANCE_FROM_PLANE;
      maxAngleFromPlane = DEFAULT_MAX_ANGLE_FROM_PLANE;
      minNormalQuality = DEFAULT_MIN_NORMAL_QUALITY;
   }

   public void set(PlanarRegionSegmentationParameters other)
   {
      searchRadius = other.searchRadius;
      maxDistanceFromPlane = other.maxDistanceFromPlane;
      maxAngleFromPlane = other.maxAngleFromPlane;
      minNormalQuality = other.minNormalQuality;
   }

   public void setSearchRadius(double searchRadius)
   {
      this.searchRadius = searchRadius;
   }

   public void setMaxDistanceFromPlane(double maxDistanceFromPlane)
   {
      this.maxDistanceFromPlane = maxDistanceFromPlane;
   }

   public void setMaxAngleFromPlane(double maxAngleFromPlane)
   {
      this.maxAngleFromPlane = maxAngleFromPlane;
   }

   public void setMinNormalQuality(double minNormalQuality)
   {
      this.minNormalQuality = minNormalQuality;
   }

   public double getSearchRadius()
   {
      return searchRadius;
   }

   public double getMaxDistanceFromPlane()
   {
      return maxDistanceFromPlane;
   }

   public double getMaxAngleFromPlane()
   {
      return maxAngleFromPlane;
   }

   public double getMinNormalQuality()
   {
      return minNormalQuality;
   }

   @Override
   public String toString()
   {
      return "search radius: " + searchRadius + ", max distance from plane: " + maxDistanceFromPlane + ", maxAngleFromPlane: " + maxAngleFromPlane + ", minNormalQuality: " + minNormalQuality;
   }

   public static PlanarRegionSegmentationParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      while (!scanner.hasNextDouble())
         scanner.next();
      double searchRadius = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double maxDistanceFromPlane = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double maxAngleFromPlane = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double minNormalQuality = scanner.nextDouble();
      scanner.close();
      PlanarRegionSegmentationParameters parameters = new PlanarRegionSegmentationParameters();
      parameters.setSearchRadius(searchRadius);
      parameters.setMaxDistanceFromPlane(maxDistanceFromPlane);
      parameters.setMaxAngleFromPlane(maxAngleFromPlane);
      parameters.setMinNormalQuality(minNormalQuality);
      return parameters;
   }
}
