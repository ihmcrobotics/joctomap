package us.ihmc.octoMap.ocTree.baseImplementation;

import us.ihmc.octoMap.node.AbstractOcTreeNode;

public interface CollidableRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public abstract boolean isCollidable(NODE node);
}
