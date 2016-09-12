package us.ihmc.octoMap.ocTree;

import us.ihmc.octoMap.node.AbstractOccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.UpdateRule;

public class SetOccupancyRule<NODE extends AbstractOccupancyOcTreeNode<NODE>> implements UpdateRule<NODE>
{
   private float newLogOdds = Float.NaN;
   private boolean lazyEvaluation = false;

   public SetOccupancyRule()
   {
   }

   public void setNewLogOdds(float newLogOdds)
   {
      this.newLogOdds = newLogOdds;
   }

   public void setLazyEvaluation(boolean lazyEvaluation)
   {
      this.lazyEvaluation = lazyEvaluation;
   }

   @Override
   public boolean doLazyEvaluation()
   {
      return lazyEvaluation;
   }
   
   @Override
   public void updateLeaf(NODE leafToUpdate)
   {

      leafToUpdate.setLogOdds(newLogOdds);
   }

   @Override
   public void updateInnerNode(NODE innerNodeToUpdate)
   {
      innerNodeToUpdate.updateOccupancyChildren();
   }
}
