package us.ihmc.octoMap.rules.interfaces;

import us.ihmc.octoMap.node.baseImplementation.AbstractOcTreeNode;

public interface IteratorSelectionRule<NODE extends AbstractOcTreeNode<NODE>>
{
   public boolean test(NODE node, int iteratorMaxDepth);
}
