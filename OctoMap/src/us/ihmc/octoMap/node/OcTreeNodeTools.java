package us.ihmc.octoMap.node;

import java.util.HashMap;

import us.ihmc.robotics.lists.GenericTypeBuilder;

/**
 * This tool class has to live in this package to be able to do operation on node's children field.
 * @author Sylvain
 *
 */
public class OcTreeNodeTools
{

   static final ThreadLocal<HashMap<Class<? extends OcTreeDataNode<?>>, GenericTypeBuilder<? extends OcTreeDataNode<?>>>> BUILDER_CACHE_THREAD_LOCAL = new ThreadLocal<HashMap<Class<? extends OcTreeDataNode<?>>, GenericTypeBuilder<? extends OcTreeDataNode<?>>>>()
   {
      @Override
      public HashMap<Class<? extends OcTreeDataNode<?>>, GenericTypeBuilder<? extends OcTreeDataNode<?>>> initialValue()
      {
         return new HashMap<>();
      }
   };

   /** 
    * Safe test if node has a child at index childIdx.
    * First tests if there are any children. Replaces node->childExists(...)
    * \return true if the child at childIdx exists
    */
   public final static boolean nodeChildExists(OcTreeDataNode<?> node, int childIndex)
   {
      checkChildIndex(childIndex);
      return node.children != null && node.children[childIndex] != null;
   }

   public final static void checkChildIndex(int childIndex)
   {
      if (childIndex > 7)
         throw new RuntimeException("Bad child index :" + childIndex + ", expected index to be in [0, 7].");
   }

   public final static void checkNodeHasChildren(OcTreeDataNode<?> node)
   {
      if (node.children == null)
         throw new RuntimeException("The given node has no children.");
   }

   public final static void checkNodeChildNotNull(OcTreeDataNode<?> node, int childIndex)
   {
      if (node.children[childIndex] == null)
         throw new RuntimeException("Child is already null.");
   }

   @SuppressWarnings("unchecked")
   public static final <V, NODE extends OcTreeDataNode<V>> NODE getNodeChild(NODE node, int childIndex)
   {
      checkChildIndex(childIndex);
      checkNodeHasChildren(node);
      checkNodeChildNotNull(node, childIndex);
      return node.children == null ? null : (NODE) node.getChildUnsafe(childIndex);
   }

}
