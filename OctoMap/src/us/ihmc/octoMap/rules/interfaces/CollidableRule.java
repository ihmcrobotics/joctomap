package us.ihmc.octoMap.rules.interfaces;

import us.ihmc.octoMap.node.baseImplementation.AbstractOcTreeNode;

public interface CollidableRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public abstract boolean isCollidable(NODE node);
}
