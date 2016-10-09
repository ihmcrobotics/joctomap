package us.ihmc.octoMap.iterators;

import java.util.ArrayDeque;
import java.util.Iterator;

import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.octoMap.rules.interfaces.IteratorSelectionRule;
import us.ihmc.octoMap.tools.OctoMapTools;

public class OcTreeIterable<NODE extends AbstractOcTreeNode<NODE>> implements Iterable<OcTreeSuperNode<NODE>>
{
   private final AbstractOcTreeBase<NODE> tree;
   private final OcTreeIterator<NODE> iterator;
   private int maxDepth;
   private IteratorSelectionRule<NODE> rule;

   OcTreeIterable(AbstractOcTreeBase<NODE> tree, int maxDepth, IteratorSelectionRule<NODE> rule, boolean recycleIterator)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;
      this.rule = rule;
      if (recycleIterator)
         iterator = new OcTreeIterator<>(tree, maxDepth, rule);
      else
         iterator = null;
   }

   @Override
   public Iterator<OcTreeSuperNode<NODE>> iterator()
   {
      if (iterator == null)
      {
         return new OcTreeIterator<>(tree, maxDepth, rule);
      }
      else
      {
         iterator.reset();
         return iterator;
      }
   }

   public void setMaxDepth(int maxDepth)
   {
      this.maxDepth = maxDepth;
      if (iterator != null)
         iterator.setMaxDepth(maxDepth);
   }

   public void setRule(IteratorSelectionRule<NODE> rule)
   {
      this.rule = rule;
      if (iterator != null)
         iterator.setRule(rule);
   }

   public static class OcTreeIterator<NODE extends AbstractOcTreeNode<NODE>> implements Iterator<OcTreeSuperNode<NODE>>
   {
      private final ArrayDeque<OcTreeSuperNode<NODE>> pool = new ArrayDeque<>();
      private IteratorSelectionRule<NODE> rule;

      /// Internal recursion stack.
      private final ArrayDeque<OcTreeSuperNode<NODE>> stack = new ArrayDeque<>();

      private AbstractOcTreeBase<NODE> tree;
      private int maxDepth; ///< Maximum depth for depth-limited queries

      private OcTreeIterator(AbstractOcTreeBase<NODE> tree, int maxDepth, IteratorSelectionRule<NODE> rule)
      {
         this.tree = tree;
         this.rule = rule;
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
            OcTreeSuperNode<NODE> superNode = getOrCreateSuperNode();
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

      public void setRule(IteratorSelectionRule<NODE> newRule)
      {
         rule = newRule;
      }

      @Override
      public boolean hasNext()
      {
         return !stack.isEmpty();
      }

      @Override
      public OcTreeSuperNode<NODE> next()
      {
         OcTreeSuperNode<NODE> currentNode = nextInternal();

         if (rule != null)
         {
            while (!rule.test(currentNode))
               currentNode = nextInternal();
         }

         return currentNode;
      }

      private OcTreeSuperNode<NODE> nextInternal()
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
         pool.add(currentNode);
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
