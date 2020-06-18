package us.ihmc.jOctoMap.tools;

import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;
import us.ihmc.jOctoMap.occupancy.OccupancyParametersReadOnly;

public abstract class OccupancyTools
{
   /**
    * @return logOddsToClip clipped to be inside [{@link #minOccupancyLogOdds},
    *         {@link #maxOccupancyLogOdds}].
    */
   public static float clipLogOddsToMinMax(OccupancyParametersReadOnly parameters, float logOddsToClip)
   {
      if (logOddsToClip < parameters.getMinLogOdds())
         return parameters.getMinLogOdds();
      else if (logOddsToClip > parameters.getMaxLogOdds())
         return parameters.getMaxLogOdds();
      else
         return logOddsToClip;
   }

   /**
    * Queries whether a node is occupied according to the tree's parameter for "occupancyThreshold"
    *
    * @param occupancyNode
    * @return
    */
   public static <NODE extends AbstractOccupancyOcTreeNode<NODE>> boolean isNodeOccupied(OccupancyParametersReadOnly parameters, NODE occupancyNode)
   {
      return occupancyNode.getLogOdds() >= parameters.getOccupancyThreshold();
   }

   /**
    * Queries whether a node is at the clamping limit according to the tree's parameter
    *
    * @param occupancyNode
    */
   public static <NODE extends AbstractOccupancyOcTreeNode<NODE>> boolean isNodeAtOccupancyLimit(OccupancyParametersReadOnly parameters, NODE occupancyNode)
   {
      return occupancyNode.getLogOdds() >= parameters.getMaxLogOdds() || occupancyNode.getLogOdds() <= parameters.getMinLogOdds();
   }

   /**
    * Update logodds value of node by adding to the current value.
    *
    * @param occupancyNode
    * @param update
    */
   public static <NODE extends AbstractOccupancyOcTreeNode<NODE>> void updateNodeLogOdds(OccupancyParametersReadOnly parameters, NODE occupancyNode,
                                                                                         float update)
   {
      occupancyNode.setLogOdds(OccupancyTools.clipLogOddsToMinMax(parameters, occupancyNode.getLogOdds() + update));
   }

   /**
    * Converts the node to the maximum likelihood occupancy value according to the tree's parameter for
    * min/max "occupancy"
    *
    * @param occupancyNode
    */
   public static <NODE extends AbstractOccupancyOcTreeNode<NODE>> void nodeToMaxLikelihood(OccupancyParametersReadOnly parameters, NODE occupancyNode)
   {
      if (isNodeOccupied(parameters, occupancyNode))
         occupancyNode.setLogOdds(parameters.getMaxLogOdds());
      else
         occupancyNode.setLogOdds(parameters.getMinLogOdds());
   }
}
