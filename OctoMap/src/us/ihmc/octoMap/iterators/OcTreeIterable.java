package us.ihmc.octoMap.iterators;

import java.util.ArrayDeque;
import java.util.Iterator;

import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.octoMap.tools.OctoMapTools;

public class OcTreeIterable<NODE extends AbstractOcTreeNode<NODE>> implements Iterable<OcTreeSuperNode<NODE>>
{
   private final AbstractOcTreeBase<NODE> tree;
   private final int maxDepth;
   private final OcTreeIterator<NODE> iterator;

   public OcTreeIterable(AbstractOcTreeBase<NODE> tree)
   {
      this(tree, 0, false);
   }

   public OcTreeIterable(AbstractOcTreeBase<NODE> tree, int maxDepth)
   {
      this(tree, maxDepth, false);
   }

   public OcTreeIterable(AbstractOcTreeBase<NODE> tree, boolean recycleIterator)
   {
      this(tree, 0, recycleIterator);
   }

   public OcTreeIterable(AbstractOcTreeBase<NODE> tree, int maxDepth, boolean recycleIterator)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;
      if (recycleIterator)
         iterator = new OcTreeIterator<>(tree, maxDepth);
      else
         iterator = null;
   }

   @Override
   public Iterator<OcTreeSuperNode<NODE>> iterator()
   {
      if (iterator == null)
      {
         return new OcTreeIterator<>(tree, maxDepth);
      }
      else
      {
         iterator.reset();
         return iterator;
      }
   }

   public static class OcTreeIterator<NODE extends AbstractOcTreeNode<NODE>> implements Iterator<OcTreeSuperNode<NODE>>
   {
      private final ArrayDeque<OcTreeSuperNode<NODE>> pool = new ArrayDeque<>();

      /// Internal recursion stack.
      private final ArrayDeque<OcTreeSuperNode<NODE>> stack = new ArrayDeque<>();

      private AbstractOcTreeBase<NODE> tree;
      private int maxDepth; ///< Maximum depth for depth-limited queries

      public OcTreeIterator(AbstractOcTreeBase<NODE> tree)
      {
         this(tree, 0);
      }

      public OcTreeIterator(AbstractOcTreeBase<NODE> tree, int maxDepth)
      {
         this.tree = tree;
         if (tree == null)
            throw new RuntimeException("Creating an iterator with no tree.");

         setMaxDepth(maxDepth);
         reset();
      }

      public void reset()
      {
         while (!stack.isEmpty())
            pool.add(stack.pop());

         if (tree.getRoot() != null)
         { // tree is not empty
            OcTreeSuperNode<NODE> superNode = new OcTreeSuperNode<>();
            superNode.setAsRootSuperNode(tree, this.maxDepth);
            stack.add(superNode);
         }
      }

      public void setMaxDepth(int maxDepth)
      {
         if (maxDepth == 0)
            this.maxDepth = tree.getTreeDepth();
         else
            this.maxDepth = maxDepth;
      }

      @Override
      public boolean hasNext()
      {
         return !stack.isEmpty();
      }

      @Override
      public OcTreeSuperNode<NODE> next()
      {
         OcTreeSuperNode<NODE> currentNode = stack.poll();

         if (currentNode.getDepth() < maxDepth)
         {
            // push on stack in reverse order
            for (int i = 7; i >= 0; i--)
            {
               if (OcTreeNodeTools.nodeChildExists(currentNode.getNode(), i))
               {
                  OcTreeSuperNode<NODE> newNode = getOrCreateSuperNode();
                  newNode.setAsChildSuperNode(currentNode, i);
                  stack.add(newNode);
                  OctoMapTools.checkIfDepthValid(newNode.getDepth(), maxDepth);
               }
            }
         }
         return currentNode;
      }

      private OcTreeSuperNode<NODE> getOrCreateSuperNode()
      {
         if (pool.isEmpty())
            return new OcTreeSuperNode<>();
         else
         {
            OcTreeSuperNode<NODE> ret = pool.pop();
            ret.clear();
            return ret;
         }
      }
   }
}
