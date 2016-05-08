package us.ihmc.octoMap;

import java.util.ArrayDeque;

import javax.vecmath.Point3d;

import us.ihmc.robotics.MathTools;

public class OcTreeIterator
{

   private static abstract class IteratorBase<V, NODE extends OcTreeDataNode<V, NODE>> // implements Iterator<NODE>
   {
      protected OcTreeBaseImpl<V, NODE> tree; ///< Octree this iterator is working on
      protected int maxDepth; ///< Maximum depth for depth-limited queries

      /// Internal recursion stack. Apparently a stack of vector works fastest here.
      protected ArrayDeque<StackElement<V, NODE>> stack = new ArrayDeque<>();

      /// Default ctor, only used for the end-iterator
      public IteratorBase()
      {
         tree = null;
         maxDepth = 0;
      }

      /**
      * Constructor of the iterator. Initializes the iterator with the default
      * constructor (= end() iterator) if tree is empty or NULL.
      *
      * @param tree OcTreeBaseImpl on which the iterator is used on
      * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
      */
      public IteratorBase(OcTreeBaseImpl<V, NODE> tree)
      {
         this(tree, 0);
      }

      public IteratorBase(OcTreeBaseImpl<V, NODE> tree, int depth)
      {
         if (tree != null && tree.root != null)
            this.tree = tree;
         maxDepth = depth;
         if (tree != null && maxDepth == 0)
            maxDepth = tree.getTreeDepth();

         if (tree != null && tree.root != null)
         { // tree is not empty
            StackElement<V, NODE> s = new StackElement<>();
            s.node = tree.root;
            s.depth = 0;
            s.key.k[0] = s.key.k[1] = s.key.k[2] = tree.tree_max_val;
            stack.add(s);
         }
         else
         { // construct the same as "end"
            tree = null;
            maxDepth = 0;
         }
      }

      /// Copy constructor of the iterator
      public IteratorBase(IteratorBase<V, NODE> other)
      {
         tree = other.tree;
         maxDepth = other.maxDepth;
         stack.addAll(other.stack);
      }

      /// Comparison between iterators. First compares the tree, then stack size and top element of stack.
      public boolean equals(IteratorBase<V, NODE> other)
      {
         if (!tree.equals(other.tree))
            return false;
         if (stack.size() != other.stack.size())
            return false;
         if (stack.isEmpty())
            return true;
         StackElement<V, NODE> thisLast = stack.peekLast();
         StackElement<V, NODE> otherLast = other.stack.peekLast();
         return thisLast.equals(otherLast);
      }

      public boolean hasReachedEnd()
      {
         return tree == null && maxDepth == 0;
      }

      /// return the center coordinate of the current node
      public Point3d getCoordinate()
      {
         return tree.keyToCoord(stack.peekLast().key, stack.peekLast().depth);
      }

      /// @return single coordinate of the current node
      public double getX()
      {
         return tree.keyToCoord(stack.peekLast().key.k[0], stack.peekLast().depth);
      }

      /// @return single coordinate of the current node
      public double getY()
      {
         return tree.keyToCoord(stack.peekLast().key.k[1], stack.peekLast().depth);
      }

      /// @return single coordinate of the current node
      public double getZ()
      {
         return tree.keyToCoord(stack.peekLast().key.k[2], stack.peekLast().depth);
      }

      /// @return the side of the volume occupied by the current node
      public double getSize()
      {
         return tree.getNodeSize(stack.peekLast().depth);
      }

      /// return depth of the current node
      public int getDepth()
      {
         return stack.peekLast().depth;
      }

      /// @return the OcTreeKey of the current node
      public OcTreeKey getKey()
      {
         return stack.peekLast().key;
      }

      public NODE getNode()
      {
         return stack.peekLast().node;
      }

      /// @return the OcTreeKey of the current node, for nodes with depth != maxDepth
      public OcTreeKey getIndexKey()
      {
         return OcTreeKey.computeIndexKey(tree.getTreeDepth() - stack.peekLast().depth, stack.peekLast().key);
      }

      /// One step of depth-first tree traversal.
      /// How this is used depends on the actual iterator.
      protected void singleIncrement()
      {
         StackElement<V, NODE> top = stack.pop();

         if (top.depth == maxDepth)
            return;

         StackElement<V, NODE> s = new StackElement<>();
         s.depth = top.depth + 1;

         int center_offset_key = tree.tree_max_val >> s.depth;
         // push on stack in reverse order
         for (int i = 7; i >= 0; --i)
         {
            if (tree.nodeChildExists(top.node, i))
            {
               OcTreeKey.computeChildKey(i, center_offset_key, top.key, s.key);
               s.node = tree.getNodeChild(top.node, i);
               //OCTOMAP_DEBUG_STR("Current depth: " << int(top.depth) << " new: "<< int(s.depth) << " child#" << i <<" ptr: "<<s.node);
               stack.push(s);
               assert (s.depth <= maxDepth);
            }
         }
      }
   }

   /**
    * Iterator over the complete tree (inner nodes and leafs).
    * See below for example usage.
    * Note that the non-trivial call to tree->end_tree() should be done only once
    * for efficiency!
    *
    * @code
    * for(OcTreeTYPE::tree_iterator it = tree->begin_tree(),
    *        end=tree->end_tree(); it!= end; ++it)
    * {
    *   //manipulate node, e.g.:
    *   std::cout << "Node center: " << it.getCoordinate() << std::endl;
    *   std::cout << "Node size: " << it.getSize() << std::endl;
    *   std::cout << "Node value: " << it->getValue() << std::endl;
    * }
    * @endcode
    */
   public static class TreeIterator<V, NODE extends OcTreeDataNode<V, NODE>> extends IteratorBase<V, NODE>
   {

      public TreeIterator()
      {
         super();
      }

      /**
      * Constructor of the iterator.
      *
      * @param tree OcTreeBaseImpl on which the iterator is used on
      * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
      */
      public TreeIterator(OcTreeBaseImpl<V, NODE> tree)
      {
         this(tree, 0);
      }

      public TreeIterator(OcTreeBaseImpl<V, NODE> tree, int depth)
      {
         super(tree, depth);
      }

      public TreeIterator(TreeIterator<V, NODE> other)
      {
         super(other);
      }

      /// postfix increment operator of iterator (it++)
      public TreeIterator<V, NODE> getAndNext()
      {
         TreeIterator<V, NODE> ret = new TreeIterator<>(this);
         this.next();
         return ret;
      }

      /// Prefix increment operator to advance the iterator
      public TreeIterator<V, NODE> nextAndGet()
      {
         if (stack.isEmpty())
            tree = null;
         else
            singleIncrement();
         return this;
      }

      public TreeIterator<V, NODE> next()
      {
         return nextAndGet();
      }

      /// @return whether the current node is a leaf, i.e. has no children or is at max level
      public boolean isLeaf()
      {
         return (!tree.nodeHasChildren(stack.peekLast().node) || stack.peekLast().depth == maxDepth);
      }
   };

   /**
    * Iterator to iterate over all leafs of the tree.
    * Inner nodes are skipped. See below for example usage.
    * Note that the non-trivial call to tree->end_leafs() should be done only once
    * for efficiency!
    *
    * @code
    * for(OcTreeTYPE::leaf_iterator it = tree->begin_leafs(),
    *        end=tree->end_leafs(); it!= end; ++it)
    * {
    *   //manipulate node, e.g.:
    *   std::cout << "Node center: " << it.getCoordinate() << std::endl;
    *   std::cout << "Node size: " << it.getSize() << std::endl;
    *   std::cout << "Node value: " << it->getValue() << std::endl;
    * }
    * @endcode
    *
    */
   public static class LeafIterator<V, NODE extends OcTreeDataNode<V, NODE>> extends IteratorBase<V, NODE>
   {

      public LeafIterator()
      {
         super();
      }

      /**
      * Constructor of the iterator.
      *
      * @param tree OcTreeBaseImpl on which the iterator is used on
      * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
      */
      public LeafIterator(OcTreeBaseImpl<V, NODE> tree)
      {
         this(tree, 0);
      }

      public LeafIterator(OcTreeBaseImpl<V, NODE> tree, int depth)
      {
         super(tree, depth);
         // tree could be empty (= no stack)
         if (stack.size() > 0)
         {
            // skip forward to next valid leaf node:
            // add root another time (one will be removed) and ++
            stack.add(stack.peekLast());
            next();
         }
      }

      public LeafIterator(LeafIterator<V, NODE> other)
      {
         super(other);
      }

      /// postfix increment operator of iterator (it++)
      public LeafIterator<V, NODE> getAndNext()
      {
         LeafIterator<V, NODE> ret = new LeafIterator<>(this);
         next();
         return ret;
      }

      /// prefix increment operator of iterator (++it)
      public LeafIterator<V, NODE> nextAndGet()
      {
         if (stack.isEmpty())
         {
            tree = null; // TODO check?
         }
         else
         {
            stack.pop();

            // skip forward to next leaf
            while (!stack.isEmpty() && stack.peekLast().depth < maxDepth && tree.nodeHasChildren(this.stack.peekLast().node))
            {
               singleIncrement();
            }
            // done: either stack is empty (== end iterator) or a next leaf node is reached!
            if (stack.isEmpty())
               tree = null;
         }

         return this;
      }

      public LeafIterator<V, NODE> next()
      {
         return nextAndGet();
      }

   }

   /**
    * Bounding-box leaf iterator. This iterator will traverse all leaf nodes
    * within a given bounding box (axis-aligned). See below for example usage.
    * Note that the non-trivial call to tree->end_leafs_bbx() should be done only once
    * for efficiency!
    *
    * @code
    * for(OcTreeTYPE::leaf_bbx_iterator it = tree->begin_leafs_bbx(min,max),
    *        end=tree->end_leafs_bbx(); it!= end; ++it)
    * {
    *   //manipulate node, e.g.:
    *   std::cout << "Node center: " << it.getCoordinate() << std::endl;
    *   std::cout << "Node size: " << it.getSize() << std::endl;
    *   std::cout << "Node value: " << it->getValue() << std::endl;
    * }
    * @endcode
    */
   public static class LeafBoundingBoxIterator<V, NODE extends OcTreeDataNode<V, NODE>> extends IteratorBase<V, NODE>
   {
      OcTreeKey minKey = new OcTreeKey();
      OcTreeKey maxKey = new OcTreeKey();

      public LeafBoundingBoxIterator()
      {
         super();
      }

      /**
      * Constructor of the iterator. The bounding box corners min and max are
      * converted into an OcTreeKey first.
      *
      * @note Due to rounding and discretization
      * effects, nodes may be traversed that have float coordinates appearing
      * outside of the (float) bounding box. However, the node's complete volume
      * will include the bounding box coordinate. For a more exact control, use
      * the constructor with OcTreeKeys instead.
      *
      * @param tree OcTreeBaseImpl on which the iterator is used on
      * @param min Minimum point3d of the axis-aligned boundingbox
      * @param max Maximum point3d of the axis-aligned boundingbox
      * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
      */
      public LeafBoundingBoxIterator(OcTreeBaseImpl<V, NODE> tree, Point3d min, Point3d max)
      {
         this(tree, min, max, 0);
      }

      public LeafBoundingBoxIterator(OcTreeBaseImpl<V, NODE> tree, Point3d min, Point3d max, int depth)
      {
         super(tree, depth);
         if (stack.size() > 0)
         {
            if (tree == null)
               throw new RuntimeException("Tree is null.");

            if (!tree.coordToKeyChecked(min, minKey) || !tree.coordToKeyChecked(max, maxKey))
            {
               // coordinates invalid, set to end iterator
               tree = null;
               maxDepth = 0;
            }
            else
            { // else: keys are generated and stored

               // advance from root to next valid leaf in bbx:
               stack.add(stack.peekLast());
               next();
            }
         }

      }

      /**
      * Constructor of the iterator. This version uses the exact keys as axis-aligned
      * bounding box (including min and max).
      *
      * @param tree OcTreeBaseImpl on which the iterator is used on
      * @param min Minimum OcTreeKey to be included in the axis-aligned boundingbox
      * @param max Maximum OcTreeKey to be included in the axis-aligned boundingbox
      * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
      */
      public LeafBoundingBoxIterator(OcTreeBaseImpl<V, NODE> tree, OcTreeKey min, OcTreeKey max)
      {
         this(tree, min, max, 0);
      }

      public LeafBoundingBoxIterator(OcTreeBaseImpl<V, NODE> tree, OcTreeKey min, OcTreeKey max, int depth)
      {
         super(tree, depth);
         minKey.set(min);
         maxKey.set(max);

         // tree could be empty (= no stack)
         if (stack.size() > 0)
         {
            // advance from root to next valid leaf in bbx:
            stack.add(stack.peekLast());
            next();
         }
      }

      public LeafBoundingBoxIterator(LeafBoundingBoxIterator<V, NODE> other)
      {
         super(other);
         minKey.set(other.minKey);
         maxKey.set(other.maxKey);
      }

      /// postfix increment operator of iterator (it++)
      public LeafBoundingBoxIterator<V, NODE> getAndNext()
      {
         LeafBoundingBoxIterator<V, NODE> ret = new LeafBoundingBoxIterator<>(this);
         next();
         return ret;
      }

      /// prefix increment operator of iterator (++it)
      public LeafBoundingBoxIterator<V, NODE> nextAndGet()
      {
         if (stack.isEmpty())
         {
            tree = null; // TODO check?

         }
         else
         {
            stack.pop();

            // skip forward to next leaf
            while (!stack.isEmpty() && stack.peekLast().depth < maxDepth && tree.nodeHasChildren(stack.peekLast().node))
            {
               singleIncrement();
            }
            // done: either stack is empty (== end iterator) or a next leaf node is reached!
            if (stack.isEmpty())
               tree = null;
         }

         return this;
      };

      public LeafBoundingBoxIterator<V, NODE> next()
      {
         return nextAndGet();
      }

      protected void singleIncrement()
      {
         StackElement<V, NODE> top = stack.pop();

         StackElement<V, NODE> s = new StackElement<>();
         s.depth = top.depth + 1;
         int center_offset_key = tree.tree_max_val >> s.depth;
         // push on stack in reverse order
         for (int i = 7; i >= 0; --i)
         {
            if (tree.nodeChildExists(top.node, i))
            {
               OcTreeKey.computeChildKey(i, center_offset_key, top.key, s.key);

               // overlap of query bbx and child bbx?
               if ((minKey.k[0] <= (s.key.k[0] + center_offset_key)) && (maxKey.k[0] >= (s.key.k[0] - center_offset_key))
                     && (minKey.k[1] <= (s.key.k[1] + center_offset_key)) && (maxKey.k[1] >= (s.key.k[1] - center_offset_key))
                     && (minKey.k[2] <= (s.key.k[2] + center_offset_key)) && (maxKey.k[2] >= (s.key.k[2] - center_offset_key)))
               {
                  s.node = tree.getNodeChild(top.node, i);
                  stack.add(s);
                  MathTools.checkIfLessOrEqual(s.depth, maxDepth);
               }
            }
         }
      }
   }

   /// Element on the internal recursion stack of the iterator
   static class StackElement<V, NODE extends OcTreeDataNode<V, NODE>>
   {
      NODE node;
      OcTreeKey key;
      int depth;

      public boolean equals(StackElement<V, NODE> other)
      {
         if (!node.equals(other.node))
            return false;
         if (!key.equals(other.key))
            return false;
         return depth == other.depth;
      }
   };
}
