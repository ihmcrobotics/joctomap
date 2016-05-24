package us.ihmc.octoMap.tools;

public class OctoMapTools
{
   /**
    * Compute log-odds from probability:
    */
   public static float logodds(double probability)
   {
      return (float) (Math.log(probability / (1.0 - probability)));
   }

   /**
    * Compute probability from logodds:
    */
   public static double probability(double logodds)
   {
      return 1.0 - (1.0 / (1.0 + Math.exp(logodds)));
   }
}
