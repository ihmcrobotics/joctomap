package us.ihmc.octoMap.node;

import us.ihmc.octoMap.tools.OctoMapTools;
import us.ihmc.robotics.MathTools;

public class OcTreeNode extends OcTreeDataNode<Float>
{

   public OcTreeNode()
   {
      this(0f);
   }

   public OcTreeNode(float initialValue)
   {
      super(initialValue);
   }

   /**
    * Return occupancy probability of node
    */
   public double getOccupancy()
   {
      return OctoMapTools.probability(value);
   }

   /**
    * Return log odds representation of occupancy probability of node
    */
   public float getLogOdds()
   {
      return value;
   }

   /**
    * Sets log odds occupancy of node
    */
   public void setLogOdds(float l)
   {
      value = l;
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
               mean += ((OcTreeNode) children[i]).getOccupancy(); // TODO check if works generally
               ++c;
            }
         }
      }

      if (c > 0)
         mean /= (double) c;

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
               float l = ((OcTreeNode) children[i]).getLogOdds(); // TODO check if works generally
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
      this.setLogOdds(this.getMaxChildLogOdds()); // conservative
   }

   /**
    * Adds p to the node's logOdds value (with no boundary / threshold checking!)
    */
   public void addValue(float logOdds)
   {
      value += logOdds;
   }

   @Override
   public boolean epsilonEquals(OcTreeDataNode<?> other)
   {
      if (!(getClass().isInstance(other)))
         return false;

      return epsilonEquals((OcTreeNode) other, 1.0e-7f);
   }

   @Override
   public boolean epsilonEquals(OcTreeDataNode<Float> other, Float epsilon)
   {
      return MathTools.epsilonEquals(value, other.value, epsilon);
   }
}
