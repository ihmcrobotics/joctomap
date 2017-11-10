package us.ihmc.jOctoMap.rules.interfaces;

import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;

public interface CollidableRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public abstract boolean isCollidable(NODE node);
}
