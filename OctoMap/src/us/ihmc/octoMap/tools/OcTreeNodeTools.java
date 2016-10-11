package us.ihmc.octoMap.tools;

import us.ihmc.octoMap.node.baseImplementation.AbstractOcTreeNode;

/**
 * This tool class has to live in this package to be able to do operation on node's children field.
 * @author Sylvain
 *
 */
public abstract class OcTreeNodeTools
{
   /** 
    * Safe test if node has a child at index childIdx.
    * First tests if there are any children. Replaces node->childExists(...)
    * \return true if the child at childIdx exists
    */
   public final static boolean nodeChildExists(AbstractOcTreeNode<?> node, int childIndex)
   {
      checkChildIndex(childIndex);
      return node.hasArrayForChildren() && node.getChild(childIndex) != null;
   }

   public final static void checkChildIndex(int childIndex)
   {
      if (childIndex > 7 || childIndex < 0)
         throw new RuntimeException("Bad child index :" + childIndex + ", expected index to be in [0, 7].");
   }

   /**
    *  A node is collapsible if all children exist, don't have children of their own
    * and have the same occupancy value
    * @param node
    * @return
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> boolean isNodeCollapsible(NODE node, double epsilon)
   {
      // All children must exist, must not have children of
      // their own and have the same occupancy probability
      if (!node.hasArrayForChildren())
         return false;
   
      NODE firstChild = node.getChild(0);
      if (firstChild == null || firstChild.hasAtLeastOneChild())
         return false;
   
      for (int i = 1; i < 8; i++)
      {
         NODE currentChild = node.getChild(i);
   
         if (currentChild == null || currentChild.hasAtLeastOneChild() || !currentChild.epsilonEquals(firstChild, epsilon))
            return false;
      }
      return true;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> int computeNumberOfLeafDescendants(NODE parent)
   {
      if (parent == null)
         throw new RuntimeException("The given parent node is null");
   
      if (!parent.hasAtLeastOneChild()) // this is a leaf -> terminate
         return 1;
   
      int sumOfLeafChildren = 0;
      for (int i = 0; i < 8; ++i)
      {
         NODE child = parent.getChild(i);
         if (child != null)
            sumOfLeafChildren += computeNumberOfLeafDescendants(child);
      }
      return sumOfLeafChildren;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> int computeNumberOfDescedants(NODE node)
   {
      if (node != null)
         return computeNumberOfNodesRecursively(node, 1);
      else
         return 0;
   }

   private static <NODE extends AbstractOcTreeNode<NODE>> int computeNumberOfNodesRecursively(NODE node, int currentNumberOfNodes)
   {
      if (node == null)
         throw new RuntimeException("The given node is null");

      if (node.hasAtLeastOneChild())
      {
         for (int i = 0; i < 8; i++)
         {
            NODE childNode = node.getChild(i);
            if (childNode != null)
            {
               currentNumberOfNodes++;
               currentNumberOfNodes = computeNumberOfNodesRecursively(childNode, currentNumberOfNodes);
            }
         }
      }

      return currentNumberOfNodes;
   }
}
