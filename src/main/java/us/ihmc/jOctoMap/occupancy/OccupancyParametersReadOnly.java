package us.ihmc.jOctoMap.occupancy;

public interface OccupancyParametersReadOnly
{

   /** @return threshold (probability) for occupancy - sensor model */
   double getOccupancyThreshold();

   /** @return threshold (logodds) for occupancy - sensor model */
   float getOccupancyThresholdLogOdds();

   /** @return probability for a "hit" in the sensor model (probability) */
   double getHitProbability();

   /** @return probability for a "hit" in the sensor model (logodds) */
   float getHitProbabilityLogOdds();

   /** @return probability for a "miss"  in the sensor model (probability) */
   double getMissProbability();

   /** @return probability for a "miss"  in the sensor model (logodds) */
   float getMissProbabilityLogOdds();

   /** @return minimum probability for occupancy clamping in the sensor model */
   double getMinProbability();

   /** @return minimum logodds for occupancy clamping in the sensor model */
   float getMinLogOdds();

   /** @return maximum probability for occupancy clamping in the sensor model */
   double getMaxProbability();

   /** @return maximum logodds for occupancy clamping in the sensor model */
   float getMaxLogOdds();

   /** @return {@link #hitUpdateLogOdds} if hit == true, {@link #missUpdateLogOdds} otherwise. */
   float getUpdateLogOdds(boolean hit);
}