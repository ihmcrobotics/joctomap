package us.ihmc.octoMap.iterators;

import java.util.ArrayDeque;
import java.util.Iterator;

import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.rules.interfaces.IteratorSelectionRule;

public class OcTreeIterable<NODE extends AbstractOcTreeNode<NODE>> implements Iterable<NODE>
{
   private NODE root;
   private int maxDepth;
   private IteratorSelectionRule<NODE> rule;

   public OcTreeIterable(NODE root)
   {
      this.root = root;
   }

   public OcTreeIterable(NODE root, IteratorSelectionRule<NODE> rule)
   {
      this.root = root;
      setRule(rule);
   }
   
   public void setMaxDepth(int maxDepth)
   {
      this.maxDepth = maxDepth;
   }

   public void setRule(IteratorSelectionRule<NODE> rule)
   {
      this.rule = rule;
   }

   @Override
   public Iterator<NODE> iterator()
   {
      return new OcTreeIterator<>(root, maxDepth, rule);
   }

   public static class OcTreeIterator<NODE extends AbstractOcTreeNode<NODE>> implements Iterator<NODE>
   {
      private final NODE root;
      private final IteratorSelectionRule<NODE> rule;

      /// Internal recursion stack.
      private final ArrayDeque<NODE> stack = new ArrayDeque<>();

      private int maxDepth; ///< Maximum depth for depth-limited queries

      private OcTreeIterator(NODE root, int maxDepth, IteratorSelectionRule<NODE> rule)
      {
         this.root = root;
         this.rule = rule;

         setMaxDepth(maxDepth);
         initialize();
      }

      private void initialize()
      {
         hasNextHasBeenCalled = false;
         stack.clear();

         if (root != null)
         { // tree is not empty
            stack.add(root);
         }
      }

      private void setMaxDepth(int maxDepth)
      {
         if (maxDepth == 0)
            this.maxDepth = Integer.MAX_VALUE;
         else
            this.maxDepth = maxDepth;
      }

      private NODE next = null;
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
      public NODE next()
      {
         if (!hasNextHasBeenCalled)
         {
            if (!hasNext())
               throw new NullPointerException();
         }

         hasNextHasBeenCalled = false;
         NODE ret = next;
         next = null;
         return ret;
      }

      private NODE searchNextNodePassingRule()
      {
         if (stack.isEmpty())
            return null;

         if (rule == null)
            return searchNextNode();

         while (!stack.isEmpty())
         {
            NODE currentNode = searchNextNode();
            if (currentNode == null || rule.test(currentNode, maxDepth))
               return currentNode;
         }
         return null;
      }

      private NODE searchNextNode()
      {
         if (stack.isEmpty())
            return null;

         NODE currentNode = stack.poll();

         if (currentNode.hasArrayForChildren() && currentNode.getDepth() < maxDepth)
         {
            // push on stack in reverse order
            for (int i = 7; i >= 0; i--)
            {
               NODE child = currentNode.getChild(i);
               if (child != null)
                  stack.add(child);
            }
         }

         return currentNode;
      }
   }
}
