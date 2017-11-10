package us.ihmc.jOctoMap.tools;

public abstract class JOctoMapTools
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

   public static void checkIfDepthValid(int depthToCheck, int treeDepth)
   {
      if (depthToCheck < 0 || depthToCheck > treeDepth)
         throw new RuntimeException("Given depth is invalid: " + depthToCheck);
   }

   public static double nanoSecondsToSeconds(long timeInNanoSeconds)
   {
      return ((double) timeInNanoSeconds) / 1e9;
   }

   public static double square(double x)
   {
      return x * x;
   }
}
