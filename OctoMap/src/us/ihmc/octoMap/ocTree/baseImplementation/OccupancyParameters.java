package us.ihmc.octoMap.ocTree.baseImplementation;

import static us.ihmc.octoMap.tools.OctoMapTools.logodds;
import static us.ihmc.octoMap.tools.OctoMapTools.probability;

public class OccupancyParameters implements OccupancyParametersReadOnly
{
   public static final double DEFAULT_OCCUPANCY_THRESHOLD = 0.5;  // = 0.0 in logodds
   public static final double DEFAULT_HIT_UPDATE = 0.7;           // = 0.85 in logodds
   public static final double DEFAULT_MISS_UPDATE = 0.4;          // = -0.4 in logodds
   public static final double DEFAULT_MIN_PROBABILITY = 0.1192;   // = -2 in log odds
   public static final double DEFAULT_MAX_PROBABILITY = 0.971;    // = 3.5 in log odds

   private float minOccupancyLogOdds;
   private float maxOccupancyLogOdds;
   private float hitUpdateLogOdds;
   private float missUpdateLogOdds;
   private float occupancyThresholdLogOdds;

   public OccupancyParameters()
   {
      setDefaultParameters();
   }

   public OccupancyParameters(OccupancyParameters other)
   {
      set(other);
   }

   public void set(OccupancyParameters other)
   {
      minOccupancyLogOdds = other.minOccupancyLogOdds;
      maxOccupancyLogOdds = other.maxOccupancyLogOdds;
      hitUpdateLogOdds = other.hitUpdateLogOdds;
      missUpdateLogOdds = other.missUpdateLogOdds;
      occupancyThresholdLogOdds = other.occupancyThresholdLogOdds;
   }

   public void setDefaultParameters()
   {
      // some sane default values:
      setOccupancyThreshold(DEFAULT_OCCUPANCY_THRESHOLD);
      setHitProbabilityUpdate(DEFAULT_HIT_UPDATE);
      setMissProbabilityUpdate(DEFAULT_MISS_UPDATE);

      setMinProbability(DEFAULT_MIN_PROBABILITY);
      setMaxProbability(DEFAULT_MAX_PROBABILITY);
   }

   /**
    * Sets the threshold for occupancy (sensor model)
    * @param probability
    */
   public void setOccupancyThreshold(double probability)
   {
      occupancyThresholdLogOdds = logodds(probability);
   }

   /**
    * Sets the probability for a "hit" (will be converted to logodds) - sensor model
    * @param probability
    */
   public void setHitProbabilityUpdate(double probability)
   {
      hitUpdateLogOdds = logodds(probability);
      if (hitUpdateLogOdds < 0.0)
         throw new RuntimeException("Invalid hit probability update: " + probability);
   }

   /**
    * Sets the probability for a "miss" (will be converted to logodds) - sensor model
    * @param probability
    */
   public void setMissProbabilityUpdate(double probability)
   {
      missUpdateLogOdds = logodds(probability);
      if (missUpdateLogOdds > 0.0)
         throw new RuntimeException("Invalid miss probability update: " + probability);
   }

   /**
    * Sets the minimum probability for occupancy clamping (sensor model)
    * @param minimumProbability
    */
   public void setMinProbability(double minimumProbability)
   {
      minOccupancyLogOdds = logodds(minimumProbability);
   }

   /**
    * Sets the maximum probability for occupancy clamping (sensor model)
    * @param maximumProbability
    */
   public void setMaxProbability(double maximumProbability)
   {
      maxOccupancyLogOdds = logodds(maximumProbability);
   }

   /** {@inheritDoc} */
   @Override
   public double getOccupancyThreshold()
   {
      return probability(occupancyThresholdLogOdds);
   }

   /** {@inheritDoc} */
   @Override
   public float getOccupancyThresholdLogOdds()
   {
      return occupancyThresholdLogOdds;
   }

   /** {@inheritDoc} */
   @Override
   public double getHitProbability()
   {
      return probability(hitUpdateLogOdds);
   }

   /** {@inheritDoc} */
   @Override
   public float getHitProbabilityLogOdds()
   {
      return hitUpdateLogOdds;
   }

   /** {@inheritDoc} */
   @Override
   public double getMissProbability()
   {
      return probability(missUpdateLogOdds);
   }

   /** {@inheritDoc} */
   @Override
   public float getMissProbabilityLogOdds()
   {
      return missUpdateLogOdds;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinProbability()
   {
      return probability(minOccupancyLogOdds);
   }

   /** {@inheritDoc} */
   @Override
   public float getMinLogOdds()
   {
      return minOccupancyLogOdds;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxProbability()
   {
      return probability(maxOccupancyLogOdds);
   }

   /** {@inheritDoc} */
   @Override
   public float getMaxLogOdds()
   {
      return maxOccupancyLogOdds;
   }

   /** {@inheritDoc} */
   @Override
   public float getUpdateLogOdds(boolean hit)
   {
      return hit ? hitUpdateLogOdds : minOccupancyLogOdds;
   }
}
