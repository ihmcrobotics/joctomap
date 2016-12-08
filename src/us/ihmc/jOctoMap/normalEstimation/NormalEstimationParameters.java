package us.ihmc.jOctoMap.normalEstimation;

import java.util.Scanner;

import us.ihmc.jOctoMap.tools.ScannerTools;

public class NormalEstimationParameters
{
   public static final double DEFAULT_SEARCH_RADIUS = 0.08;
   public static final double DEFAULT_MAX_DISTANCE_FROM_PLANE = 0.03;
   public static final double DEFAULT_MIN_CONSENSUS_RATIO = 0.5;
   public static final double DEFAULT_MAX_AVERAGE_DEVIATION_RATIO = 0.75;

   private double searchRadius;
   private double maxDistanceFromPlane;

   private double minConsensusRatio;
   private double maxAverageDeviationRatio;

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
      minConsensusRatio = DEFAULT_MIN_CONSENSUS_RATIO;
      maxAverageDeviationRatio = DEFAULT_MAX_AVERAGE_DEVIATION_RATIO;
   }

   public void set(NormalEstimationParameters other)
   {
      searchRadius = other.searchRadius;
      maxDistanceFromPlane = other.maxDistanceFromPlane;
      minConsensusRatio = other.minConsensusRatio;
      maxAverageDeviationRatio = other.maxAverageDeviationRatio;
   }

   public void setSearchRadius(double searchRadius)
   {
      this.searchRadius = searchRadius;
   }

   public void setMaxDistanceFromPlane(double maxDistanceFromPlane)
   {
      this.maxDistanceFromPlane = maxDistanceFromPlane;
   }

   public void setMinConsensusRatio(double minConsensusRatio)
   {
      this.minConsensusRatio = minConsensusRatio;
   }

   public void setMaxAverageDeviationRatio(double maxAverageDeviationRatio)
   {
      this.maxAverageDeviationRatio = maxAverageDeviationRatio;
   }

   public double getSearchRadius()
   {
      return searchRadius;
   }

   public double getMaxDistanceFromPlane()
   {
      return maxDistanceFromPlane;
   }

   public double getMinConsensusRatio()
   {
      return minConsensusRatio;
   }

   public double getMaxAverageDeviationRatio()
   {
      return maxAverageDeviationRatio;
   }

   @Override
   public String toString()
   {
      return "search radius: " + searchRadius + ", max distance from plane: " + maxDistanceFromPlane + ", min consensus ratio: " + minConsensusRatio + ", max average deviation ratio: " + maxAverageDeviationRatio;
   }

   public static NormalEstimationParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      NormalEstimationParameters parameters = new NormalEstimationParameters();
      parameters.setSearchRadius(ScannerTools.readNextDouble(scanner, parameters.getSearchRadius()));
      parameters.setMaxDistanceFromPlane(ScannerTools.readNextDouble(scanner, parameters.getMaxDistanceFromPlane()));
      parameters.setMinConsensusRatio(ScannerTools.readNextDouble(scanner, parameters.getMinConsensusRatio()));
      parameters.setMaxAverageDeviationRatio(ScannerTools.readNextDouble(scanner, parameters.getMaxAverageDeviationRatio()));
      scanner.close();
      return parameters;
   }
}
