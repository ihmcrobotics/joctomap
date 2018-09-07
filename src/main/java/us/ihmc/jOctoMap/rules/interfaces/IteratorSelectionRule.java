package us.ihmc.jOctoMap.rules.interfaces;

import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;

public interface IteratorSelectionRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public boolean test(NODE node, int iteratorMaxDepth);
}
