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
         hasNextHasBeenCalled = false;
         stack.clear();

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

      public void setRule(IteratorSelectionRule<NODE> newRule)
      {
         rule = newRule;
      }

      private OcTreeSuperNode<NODE> next = null;
      private boolean hasNextHasBeenCalled = false;

      @Override
      public boolean hasNext()
      {
         next = null;

         if (stack.isEmpty())
            return false;

         if (!hasNextHasBeenCalled)
         {
            next = searchNextNodePassingRule();
            hasNextHasBeenCalled = true;
         }

         return next != null;
      }

      @Override
      public OcTreeSuperNode<NODE> next()
      {
         if (!hasNextHasBeenCalled)
         {
            if (!hasNext())
               throw new NullPointerException();
         }

         hasNextHasBeenCalled = false;
         OcTreeSuperNode<NODE> ret = next;
         next = null;
         return ret;
      }

      private OcTreeSuperNode<NODE> searchNextNodePassingRule()
      {
         if (stack.isEmpty())
            return null;

         if (rule == null)
            return searchNextNode();

         while (!stack.isEmpty())
         {
            OcTreeSuperNode<NODE> currentNode = searchNextNode();
            if (currentNode == null || rule.test(currentNode))
               return currentNode;
         }
         return null;
      }

      private OcTreeSuperNode<NODE> searchNextNode()
      {
         if (stack.isEmpty())
            return null;

         OcTreeSuperNode<NODE> currentNode = stack.poll();

         if (currentNode.getDepth() < maxDepth)
         {
            // push on stack in reverse order
            for (int i = 7; i >= 0; i--)
            {
               if (OcTreeNodeTools.nodeChildExists(currentNode.getNode(), i))
               {
                  OcTreeSuperNode<NODE> newNode = new OcTreeSuperNode<>();
                  newNode.setAsChildSuperNode(currentNode, i);
                  stack.add(newNode);
                  OctoMapTools.checkIfDepthValid(newNode.getDepth(), maxDepth);
               }
            }
         }

         return currentNode;
      }
   }
}
