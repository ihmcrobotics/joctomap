package us.ihmc.jOctoMap.node.baseImplementation;

import java.util.Arrays;

import us.ihmc.jOctoMap.tools.JOctoMapTools;

public abstract class AbstractOccupancyOcTreeNode<N extends AbstractOccupancyOcTreeNode<N>> extends AbstractOcTreeNode<N>
{
   float logOdds;

   public AbstractOccupancyOcTreeNode()
   {
   }

   @Override
   public void copyData(N other)
   {
      logOdds = other.logOdds;
   }

   public void resetLogOdds()
   {
      logOdds = 0;
   }

   /**
    * Return occupancy probability of node
    */
   public double getOccupancy()
   {
      return JOctoMapTools.probability(logOdds);
   }

   /**
    * Return log odds representation of occupancy probability of node
    */
   public float getLogOdds()
   {
      return logOdds;
   }

   /**
    * Sets log odds occupancy of node
    */
   public void setLogOdds(float l)
   {
      logOdds = l;
   }

   /**
    * @return mean of all children's occupancy probabilities, in log odds
    */
   public double getMeanChildLogOdds()
   {
      double mean = 0;
      int c = 0;
      if (children != null)
      {
         for (int i = 0; i < 8; i++)
         {
            if (children[i] != null)
            {
               mean += children[i].getOccupancy(); // TODO check if works generally
               ++c;
            }
         }
      }

      if (c > 0)
         mean /= c;

      return Math.log(mean / (1 - mean));
   }

   /**
    * @return maximum of children's occupancy probabilities, in log odds
    */
   public float getMaxChildLogOdds()
   {
      float max = Float.NEGATIVE_INFINITY;

      if (children != null)
      {
         for (int i = 0; i < 8; i++)
         {
            if (children[i] != null)
            {
               float l = children[i].getLogOdds(); // TODO check if works generally
               if (l > max)
                  max = l;
            }
         }
      }
      return max;
   }

   /**
    * Update this node's occupancy according to its children's maximum occupancy
    */
   public void updateOccupancyChildren()
   {
      setLogOdds(getMaxChildLogOdds()); // conservative
   }

   /**
    * Adds p to the node's logOdds value (with no boundary / threshold checking!)
    */
   public void addValue(float logOdds)
   {
      this.logOdds += logOdds;
   }

   @Override
   protected boolean epsilonEqualsInternal(N other, double epsilon)
   {
      return Math.abs(logOdds - other.logOdds) <= epsilon;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": logOdds = " + logOdds + ", children = " + Arrays.toString(getChildrenSimpleNames());
   }
}
