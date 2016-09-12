package us.ihmc.octoMap.node;

import java.util.HashMap;

import us.ihmc.octoMap.node.AbstractOcTreeNode;

public class NodeManager<NODE extends AbstractOcTreeNode<NODE>>
{
   static final ThreadLocal<HashMap<Class<? extends AbstractOcTreeNode<?>>, NodeBuilder<? extends AbstractOcTreeNode<?>>>> BUILDER_CACHE_THREAD_LOCAL = new ThreadLocal<HashMap<Class<? extends AbstractOcTreeNode<?>>, NodeBuilder<? extends AbstractOcTreeNode<?>>>>()
   {
      @Override
      public HashMap<Class<? extends AbstractOcTreeNode<?>>, NodeBuilder<? extends AbstractOcTreeNode<?>>> initialValue()
      {
         return new HashMap<>();
      }
   };

   private final NodeBuilder<NODE> nodeBuilder;
   

   public NodeManager(Class<NODE> nodeClass)
   {
      nodeBuilder = new NodeBuilder<>(nodeClass);
   }
}
