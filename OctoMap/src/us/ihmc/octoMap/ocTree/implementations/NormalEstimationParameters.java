package us.ihmc.octoMap.ocTree.implementations;

import java.util.Scanner;

public class NormalEstimationParameters
{
   public static final double DEFAULT_SEARCH_RADIUS = 0.08;
   public static final double DEFAULT_MAX_DISTANCE_FROM_PLANE = 0.03;

   private double searchRadius;
   private double maxDistanceFromPlane;

   public NormalEstimationParameters()
   {
      setDefaultParameters();
   }

   public NormalEstimationParameters(NormalEstimationParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      searchRadius = DEFAULT_SEARCH_RADIUS;
      maxDistanceFromPlane = DEFAULT_MAX_DISTANCE_FROM_PLANE;
   }

   public void set(NormalEstimationParameters other)
   {
      searchRadius = other.searchRadius;
      maxDistanceFromPlane = other.maxDistanceFromPlane;
   }

   public void setSearchRadius(double searchRadius)
   {
      this.searchRadius = searchRadius;
   }

   public void setMaxDistanceFromPlane(double maxDistanceFromPlane)
   {
      this.maxDistanceFromPlane = maxDistanceFromPlane;
   }

   public double getSearchRadius()
   {
      return searchRadius;
   }

   public double getMaxDistanceFromPlane()
   {
      return maxDistanceFromPlane;
   }

   @Override
   public String toString()
   {
      return "search radius: " + searchRadius + ", max distance from plane: " + maxDistanceFromPlane;
   }

   public static NormalEstimationParameters parseNormalEstimationParameters(String normalEstimationParametersAsString)
   {
      normalEstimationParametersAsString = normalEstimationParametersAsString.replace(",", "");
      Scanner scanner = new Scanner(normalEstimationParametersAsString);
      while (!scanner.hasNextDouble())
         scanner.next();
      double searchRadius = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double maxDistanceFromPlane = scanner.nextDouble();
      scanner.close();
      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setSearchRadius(searchRadius);
      normalEstimationParameters.setMaxDistanceFromPlane(maxDistanceFromPlane);
      return normalEstimationParameters;
   }
}
