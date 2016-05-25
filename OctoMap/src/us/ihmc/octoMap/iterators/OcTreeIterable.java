package us.ihmc.octoMap.iterators;

import java.util.ArrayDeque;
import java.util.Iterator;

import us.ihmc.octoMap.OcTreeBaseImpl;
import us.ihmc.octoMap.node.OcTreeDataNode;
import us.ihmc.robotics.MathTools;

public class OcTreeIterable<NODE extends OcTreeDataNode<?>> implements Iterable<OcTreeSuperNode<NODE>>
{
   private final OcTreeBaseImpl<?, NODE> tree;
   private final int maxDepth;

   public OcTreeIterable(OcTreeBaseImpl<?, NODE> tree)
   {
      this(tree, 0);
   }

   public OcTreeIterable(OcTreeBaseImpl<?, NODE> tree, int maxDepth)
   {
      this.tree = tree;
      this.maxDepth = maxDepth;
   }

   @Override
   public Iterator<OcTreeSuperNode<NODE>> iterator()
   {
      return new OcTreeIterator<>(tree, maxDepth);
   }

   public static class OcTreeIterator<NODE extends OcTreeDataNode<?>> implements Iterator<OcTreeSuperNode<NODE>>
   {
      private final OcTreeBaseImpl<?, NODE> tree; ///< Octree this iterator is working on
      private final int maxDepth; ///< Maximum depth for depth-limited queries

      /// Internal recursion stack.
      private final ArrayDeque<OcTreeSuperNode<NODE>> stack = new ArrayDeque<>();

      public OcTreeIterator(OcTreeBaseImpl<?, NODE> tree)
      {
         this(tree, 0);
      }

      public OcTreeIterator(OcTreeBaseImpl<?, NODE> tree, int maxDepth)
      {
         if (tree == null)
            throw new RuntimeException("Creating an iterator with no tree.");

         this.tree = tree;
         if (maxDepth == 0)
            this.maxDepth = tree.getTreeDepth();
         else
            this.maxDepth = maxDepth;

         if (tree.getRoot() != null)
         { // tree is not empty
            stack.add(new OcTreeSuperNode<>(tree, maxDepth));
         }
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
               if (OcTreeDataNode.nodeChildExists(currentNode.getNode(), i))
               {
                  OcTreeSuperNode<NODE> newNode = new OcTreeSuperNode<>(tree, currentNode, i, maxDepth);
                  stack.add(newNode);
                  MathTools.checkIfLessOrEqual(newNode.getDepth(), maxDepth);
               }
            }
         }
         return currentNode;
      }
   }
}
