package us.ihmc.jOctoMap.ocTree.baseImplementation;

import static us.ihmc.jOctoMap.tools.OcTreeNodeTools.*;

import java.util.ArrayDeque;
import java.util.Iterator;
import java.util.Queue;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.NodeBuilder;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;
import us.ihmc.jOctoMap.rules.interfaces.EarlyAbortRule;
import us.ihmc.jOctoMap.rules.interfaces.UpdateRule;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.jOctoMap.tools.OcTreeKeyTools;
import us.ihmc.jOctoMap.tools.OcTreeNodeTools;
import us.ihmc.jOctoMap.tools.OcTreeSearchTools;

/**
 * OcTree base class, to be used with with any kind of OcTreeDataNode.
 *
 * This tree implementation currently has a maximum depth of 16
 * nodes. For this reason, coordinates values have to be, e.g.,
 * below +/- 327.68 meters (2^15) at a maximum resolution of 0.01m.
 *
 * This limitation enables the use of an efficient key generation
 * method which uses the binary representation of the data point
 * coordinates.
 *
 * \note You should probably not use this class directly, but
 * OcTreeBase or OccupancyOcTreeBase instead
 *
 * \tparam NODE Node class to be used in tree (usually derived from
 *    OcTreeDataNode)
 */
public abstract class AbstractOcTreeBase<NODE extends AbstractOcTreeNode<NODE>> implements Iterable<NODE>
{
   private static final int MAX_TREE_DEPTH = 30;
   private static final boolean RECYCLE_NODES = false;

   protected NODE root; ///< root NODE, null for empty tree
   private final NodeBuilder<NODE> nodeBuilder;
   private final Queue<NODE> unusedNodes = new ArrayDeque<>(1000);
   private final Queue<NODE[]> unusedNodeArrays = new ArrayDeque<>(1000 / 8);

   // constants of the tree
   /** Maximum tree depth (fixed to 16 usually) */
   protected final int treeDepth;
   protected final double resolution; ///< in meters

   protected int treeSize; ///< number of nodes in tree
   /** flag to denote whether the octree extent changed (for lazy min/max eval) */
   protected boolean sizeChanged;

   /// data structure for ray casting, array for multithreading

   public AbstractOcTreeBase(double resolution)
   {
      this(resolution, 16);
   }

   /// Constructor to enable derived classes to change tree constants.
   /// This usually requires a re-implementation of some core tree-traversal functions as well!
   protected AbstractOcTreeBase(double resolution, int treeDepth)
   {
      root = null;
      this.resolution = resolution;
      if (treeDepth > MAX_TREE_DEPTH)
         throw new RuntimeException("Cannot create a tree with a depth greater than: " + MAX_TREE_DEPTH);
      this.treeDepth = treeDepth;
      treeSize = 0;
      nodeBuilder = new NodeBuilder<>(getNodeClass());
   }

   public AbstractOcTreeBase(AbstractOcTreeBase<NODE> other)
   {
      resolution = other.resolution;
      treeDepth = other.treeDepth;
      nodeBuilder = new NodeBuilder<>(getNodeClass());
      MutableInt mutableTreeSize = new MutableInt(0);
      if (other.root != null)
         root = other.root.cloneRecursive(nodeBuilder, mutableTreeSize);
      treeSize = mutableTreeSize.intValue();
   }

   /**
    * Swap contents of two octrees, i.e., only the underlying
    * pointer / tree structure. You have to ensure yourself that the
    * metadata (resolution etc) matches. No memory is cleared
    * in this function
    */
   public void swapContent(AbstractOcTreeBase<NODE> other)
   {
      NODE thisRoot = root;
      root = other.root;
      other.root = thisRoot;

      int thisSize = treeSize;
      treeSize = other.treeSize;
      other.treeSize = thisSize;
   }

   /// Comparison between two octrees, all meta data, all
   /// nodes, and the structure must be identical
   public boolean epsilonEquals(AbstractOcTreeBase<NODE> other, double epsilon)
   {
      if (treeDepth != other.treeDepth || resolution != other.resolution || treeSize != other.treeSize)
         return false;

      // traverse all nodes, check if structure the same
      Iterator<NODE> thisIterator = OcTreeIteratorFactory.createIteratable(root).iterator();
      Iterator<NODE> otherIterator = OcTreeIteratorFactory.createIteratable(other.root).iterator();

      for (NODE thisNode = thisIterator.next(), otherNode = otherIterator.next(); thisIterator.hasNext(); thisNode = thisIterator.next(), otherNode = otherIterator.next())
      {
         if (!otherIterator.hasNext()) // The other tree has less nodes
            return false;
         if (!thisNode.epsilonEquals(otherNode, epsilon))
            return false;
      }

      if (otherIterator.hasNext()) // The other tree has more nodes
         return false;

      return true;
   }

   public double getResolution()
   {
      return resolution;
   }

   public int getTreeDepth()
   {
      return treeDepth;
   }

   // -- Tree structure operations formerly contained in the nodes ---

   /// Creates (allocates) the i-th child of the node. @return ptr to newly create NODE
   protected NODE createNodeChild(NODE node, int childIndex, int childDepth)
   {
      checkChildIndex(childIndex);
      assignChildrenArrayIfNecessary(node);

      if (OcTreeNodeTools.nodeChildExists(node, childIndex))
         throw new RuntimeException("Something went wrong.");

      OcTreeKey childKey = OcTreeKeyTools.computeChildKey(childIndex, node, childDepth, treeDepth);
      NODE newChildNode = getOrCreateNode(childKey, childDepth);

      node.setChild(childIndex, newChildNode);

      treeSize++;
      sizeChanged = true;

      return newChildNode;
   }

   private void assignChildrenArrayIfNecessary(NODE node)
   {
      if (!node.hasArrayForChildren())
      {
         if (RECYCLE_NODES)
         {
            if (unusedNodeArrays.isEmpty())
               node.allocateChildren();
            else
               node.assignChildren(unusedNodeArrays.poll());
         }
         else
         {
            node.allocateChildren();
         }
      }
   }

   private NODE getOrCreateNode(OcTreeKeyReadOnly nodeKey, int nodeDepth)
   {
      NODE newNode;
      if (RECYCLE_NODES)
         newNode = unusedNodes.isEmpty() ? nodeBuilder.createNode() : unusedNodes.poll();
      else
         newNode = nodeBuilder.createNode();
      newNode.setProperties(nodeKey, nodeDepth, resolution, treeDepth);
      return newNode;
   }

   /// Deletes the i-th child of the node
   public void deleteNodeChild(NODE node, int childIndex)
   {
      if (!nodeChildExists(node, childIndex))
         return;

      if (RECYCLE_NODES)
         unusedNodes.add(node.removeChild(childIndex));
      else
         node.removeChild(childIndex);

      treeSize--;
      sizeChanged = true;
   }

   /**
    * Generic method to search down a node to update using the {@link UpdateRule#updateLeaf(AbstractOcTreeNode)}.
    * <p>
    * If {@link UpdateRule#doLazyEvaluation()} returns false, the parents of the updated node will be updated using {@link UpdateRule#updateInnerNode(AbstractOcTreeNode)}.
    * This only works if key is at the lowest octree level.
    * @param coordinate 3d coordinate of the NODE that is to be updated
    * @param updateRule Specifies how the NODE and its parents should be updated.
    * @param earlyAbortRule (can be null) specifies edge cases for which, it is not necessary to update the NODE chain down to the lowest level. (for instance, the update would not change anything.)
    * @return the updated NODE
    */
   protected NODE updateNodeInternal(Point3f coordinate, UpdateRule<NODE> updateRule, EarlyAbortRule<NODE> earlyAbortRule)
   {
      return updateNodeInternal(coordinate.getX(), coordinate.getY(), coordinate.getZ(), updateRule, earlyAbortRule);
   }

   /**
    * Generic method to search down a node to update using the {@link UpdateRule#updateLeaf(AbstractOcTreeNode)}.
    * <p>
    * If {@link UpdateRule#doLazyEvaluation()} returns false, the parents of the updated node will be updated using {@link UpdateRule#updateInnerNode(AbstractOcTreeNode)}.
    * This only works if key is at the lowest octree level.
    * @param coordinate 3d coordinate of the NODE that is to be updated
    * @param updateRule Specifies how the NODE and its parents should be updated.
    * @param earlyAbortRule (can be null) specifies edge cases for which, it is not necessary to update the NODE chain down to the lowest level. (for instance, the update would not change anything.)
    * @return the updated NODE
    */
   protected NODE updateNodeInternal(Point3d coordinate, UpdateRule<NODE> updateRule, EarlyAbortRule<NODE> earlyAbortRule)
   {
      return updateNodeInternal(coordinate.getX(), coordinate.getY(), coordinate.getZ(), updateRule, earlyAbortRule);
   }

   /**
    * Generic method to search down a node to update using the {@link UpdateRule#updateLeaf(AbstractOcTreeNode)}.
    * <p>
    * If {@link UpdateRule#doLazyEvaluation()} returns false, the parents of the updated node will be updated using {@link UpdateRule#updateInnerNode(AbstractOcTreeNode)}.
    * This only works if key is at the lowest octree level.
    * @param x coordinate of the NODE that is to be updated
    * @param y coordinate of the NODE that is to be updated
    * @param z coordinate of the NODE that is to be updated
    * @param updateRule Specifies how the NODE and its parents should be updated.
    * @param earlyAbortRule (can be null) specifies edge cases for which, it is not necessary to update the NODE chain down to the lowest level. (for instance, the update would not change anything.)
    * @return the updated NODE
    */
   protected NODE updateNodeInternal(double x, double y, double z, UpdateRule<NODE> updateRule, EarlyAbortRule<NODE> earlyAbortRule)
   {
      OcTreeKey key = coordinateToKey(x, y, z);
      if (key == null)
         return null;
      else
         return updateNodeInternal(key, updateRule, earlyAbortRule);
   }

   /**
    * Generic method to search down a node to update using the {@link UpdateRule#updateLeaf(AbstractOcTreeNode)}.
    * <p>
    * If {@link UpdateRule#doLazyEvaluation()} returns false, the parents of the updated node will be updated using {@link UpdateRule#updateInnerNode(AbstractOcTreeNode)}.
    * This only works if key is at the lowest octree level.
    * @param key OcTreeKey of the NODE that is to be updated
    * @param updateRule Specifies how the NODE and its parents should be updated.
    * @param earlyAbortRule (can be null) specifies edge cases for which, it is not necessary to update the NODE chain down to the lowest level. (for instance, the update would not change anything.)
    * @return the updated NODE
    */
   protected NODE updateNodeInternal(OcTreeKeyReadOnly key, UpdateRule<NODE> updateRule, EarlyAbortRule<NODE> earlyAbortRule)
   {
      boolean createdRoot = false;

      if (root == null)
      {
         root = getOrCreateNode(OcTreeKeyTools.getRootKey(treeDepth), 0);
         treeSize++;
         sizeChanged = true;
         createdRoot = true;
      }

      if (earlyAbortRule != null)
      {
         NODE leaf = search(key);

         if (earlyAbortRule.shouldAbortFullDepthUpdate(leaf))
            return leaf;
      }

      return updateNodeRecursively(root, createdRoot, key, updateRule, 0);
   }

   /**
    * Expands a node (reverse of pruning): All children are created and
    * their occupancy probability is set to the node's value.
    *
    * You need to verify that this is indeed a pruned node (i.e. not a
    * leaf at the lowest level)
    * @param depth 
    *
    */
   public void expandNode(NODE node, int depth)
   {
      if (node.hasAtLeastOneChild())
         throw new RuntimeException("Node has already been expanded.");

      for (int k = 0; k < 8; k++)
      {
         NODE newNode = createNodeChild(node, k, depth + 1);
         newNode.copyData(node);
      }
   }

   /**
    * Prunes a node when it is collapsible
    * @return true if pruning was successful
    */
   public boolean pruneNode(NODE node)
   {
      return pruneNode(node, 1.0e-7);
   }

   public boolean pruneNode(NODE node, double epsilon)
   {
      if (!OcTreeNodeTools.isNodeCollapsible(node, epsilon))
         return false;

      // set value to children's values (all assumed equal)
      node.copyData(node.getChild(0));

      // delete children (known to be leafs at this point!)
      for (int childIndex = 0; childIndex < 8; childIndex++)
         deleteNodeChild(node, childIndex);
      if (RECYCLE_NODES)
         unusedNodeArrays.add(node.removeChildren());
      else
         node.removeChildren();

      return true;
   }

   // --------

   /**
    * \return Pointer to the root node of the tree. This pointer
    * should not be modified or deleted externally, the OcTree
    * manages its memory itself. In an empty tree, root is NULL.
    */
   public NODE getRoot()
   {
      return root;
   }

   /**
    *  Search node at specified depth given a 3d point (depth=0: search full tree depth)
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public NODE search(Point3d coord)
   {
      return OcTreeSearchTools.search(root, coord, resolution, treeDepth);
   }

   public NODE search(Point3d coord, int depth)
   {
      return OcTreeSearchTools.search(root, coord, depth, resolution, treeDepth);
   }

   /**
    *  Search a node at specified depth given an addressing key (depth=0: search full tree depth)
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public NODE search(OcTreeKeyReadOnly key)
   {
      return OcTreeSearchTools.search(root, key, treeDepth);
   }

   public NODE search(OcTreeKeyReadOnly key, int depth)
   {
      return OcTreeSearchTools.search(root, key, depth, treeDepth);
   }

   /** 
    *  Delete a node (if exists) given an addressing key. Will always
    *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
    *  Pruned nodes at level "depth" will directly be deleted as a whole.
    * @param deletionRule 
    */
   public boolean deleteNode(OcTreeKeyReadOnly key)
   {
      return deleteNode(key, 0);
   }

   public boolean deleteNode(OcTreeKeyReadOnly key, int depth)
   {
      if (root == null)
         return true;

      if (depth == 0)
         depth = treeDepth;

      return deleteNodeRecursively(root, 0, depth, key);
   }

   /// Deletes the complete tree structure
   public void clear()
   {
      if (root != null)
      {
         deleteNodeRecursively(root);
         root = null;
         treeSize = 0;
         // max extent of tree changed:
         sizeChanged = true;
      }
   }

   /**
    * Lossless compression of the octree: A node will replace all of its eight
    * children if they have identical values. You usually don't have to call
    * prune() after a regular occupancy update, updateNode() incrementally
    * prunes all affected nodes.
    */
   public void prune()
   {
      if (root == null)
         return;

      for (int depth = treeDepth - 1; depth > 0; depth--)
      {
         int numberOfPrunedNodes = pruneRecursively(root, 0, depth, 0);
         if (numberOfPrunedNodes == 0)
            break;
      }
   }

   /**
    *  Expands all pruned nodes (reverse of prune())
    *  NOTE This is an expensive operation, especially when the tree is nearly empty!
    */
   public void expand()
   {
      if (root != null)
         expandRecursively(root, 0, treeDepth);
   }

   // -- statistics  ----------------------

   /// \return The number of nodes in the tree
   public int size()
   {
      return treeSize;
   }

   /// Traverses the tree to calculate the total number of nodes
   public int getNumberOfNodes()
   {
      if (root == null)
         return 0;
      else
         return OcTreeNodeTools.computeNumberOfDescedants(root);
   }

   /// Traverses the tree to calculate the total number of leaf nodes
   public int getNumberOfLeafNodes()
   {
      if (root == null)
         return 0;
      else
         return OcTreeNodeTools.computeNumberOfLeafDescendants(root);
   }

   // -- access tree nodes  ------------------

   @Override
   public Iterator<NODE> iterator()
   {
      return OcTreeIteratorFactory.createLeafIteratable(root).iterator();
   }

   //
   // Key / coordinate conversion functions
   //

   /** Converts from a 3D coordinate into a 3D addressing key */
   public OcTreeKey coordinateToKey(Point3d coord)
   {
      return OcTreeKeyConversionTools.coordinateToKey(coord, resolution, treeDepth);
   }

   /** Converts from a 3D coordinate into a 3D addressing key */
   public OcTreeKey coordinateToKey(double x, double y, double z)
   {
      return OcTreeKeyConversionTools.coordinateToKey(x, y, z, resolution, treeDepth);
   }

   /** Converts from a 3D coordinate into a 3D addressing key at a given depth */
   public OcTreeKey coordinateToKey(Point3d coord, int depth)
   {
      return OcTreeKeyConversionTools.coordinateToKey(coord, depth, resolution, treeDepth);
   }

   /** Converts from a 3D coordinate into a 3D addressing key at a given depth */
   public OcTreeKey coordinateToKey(double x, double y, double z, int depth)
   {
      return OcTreeKeyConversionTools.coordinateToKey(x, y, z, depth, resolution, treeDepth);
   }

   public boolean coordinateToKey(Point3d coord, OcTreeKey keyToPack)
   {
      return OcTreeKeyConversionTools.coordinateToKey(coord, resolution, treeDepth, keyToPack);
   }

   /** converts from a discrete key at a given depth into a coordinate corresponding to the key's center */
   public double keyToCoordinate(int key, int depth)
   {
      return OcTreeKeyConversionTools.keyToCoordinate(key, depth, resolution, treeDepth);
   }

   /** converts from a discrete key at the lowest tree level into a coordinate corresponding to the key's center */
   public double keyToCoordinate(int key)
   {
      return OcTreeKeyConversionTools.keyToCoordinate(key, resolution, treeDepth);
   }

   /** converts from an addressing key at the lowest tree level into a coordinate corresponding to the key's center */
   public Point3d keyToCoordinate(OcTreeKeyReadOnly key)
   {
      return OcTreeKeyConversionTools.keyToCoordinate(key, resolution, treeDepth);
   }

   /** converts from an addressing key at a given depth into a coordinate corresponding to the key's center */
   public Point3d keyToCoordinate(OcTreeKeyReadOnly key, int depth)
   {
      return OcTreeKeyConversionTools.keyToCoordinate(key, depth, resolution, treeDepth);
   }

   /** converts from an addressing key at the lowest tree level into a coordinate corresponding to the key's center */
   public void keyToCoordinate(OcTreeKeyReadOnly key, Point3d coordinateToPack)
   {
      OcTreeKeyConversionTools.keyToCoordinate(key, coordinateToPack, resolution, treeDepth);
   }

   /** converts from an addressing key at a given depth into a coordinate corresponding to the key's center */
   public void keyToCoordinate(OcTreeKeyReadOnly key, Point3d coordinateToPack, int depth)
   {
      OcTreeKeyConversionTools.keyToCoordinate(key, depth, coordinateToPack, resolution, treeDepth);
   }

   /// recursive delete of node and all children (deallocates memory)
   private void deleteNodeRecursively(NODE node)
   {
      if (node.hasAtLeastOneChild())
      {
         for (int i = 0; i < 8; i++)
         {
            NODE child = node.removeChild(i);
            if (child != null)
            {
               if (RECYCLE_NODES)
                  unusedNodes.add(child);
               deleteNodeRecursively(child);
            }
         }

         if (RECYCLE_NODES)
            unusedNodeArrays.add(node.removeChildren());
         else
            node.removeChildren();
      } // else: node has no children
   }

   /// recursive call of deleteNode()
   private boolean deleteNodeRecursively(NODE node, int depth, int maxDepth, OcTreeKeyReadOnly key)
   {
      if (depth >= maxDepth) // on last level: delete child when going up
         return true;

      if (node == null)
         throw new RuntimeException("The given node is null");
      if (root == null)
         throw new RuntimeException("The root node is null");

      int childIndex = OcTreeKeyTools.computeChildIndex(key, depth, treeDepth);

      if (!OcTreeNodeTools.nodeChildExists(node, childIndex))
      {
         // child does not exist, but maybe it's a pruned node?
         if (!node.hasAtLeastOneChild() && node != root)
         { // current node does not have children AND it's not the root node -> expand pruned node
            expandNode(node, depth); // tree_size and size_changed adjusted in createNodeChild(...)
         }
         else
         { // no branch here, node does not exist
            return false;
         }
      }

      // follow down further, fix inner nodes on way back up
      boolean deleteChild = deleteNodeRecursively(node.getChild(childIndex), depth + 1, maxDepth, key);
      if (deleteChild)
      {
         deleteNodeChild(node, childIndex);

         if (!node.hasAtLeastOneChild())
            return true;
      }
      // node did not lose a child, or still has other children
      return false;
   }

   private NODE updateNodeRecursively(NODE node, boolean nodeJustCreated, OcTreeKeyReadOnly key, UpdateRule<NODE> updateRule, int depth)
   {
      boolean createdNode = false;

      if (node == null)
         throw new RuntimeException("The given node is null.");

      // follow down to last level
      if (depth < treeDepth)
      {
         int childIndex = OcTreeKeyTools.computeChildIndex(key, depth, treeDepth);
         if (!OcTreeNodeTools.nodeChildExists(node, childIndex))
         {
            if (!updateRule.enableNodeCreation())
            {
               updateRule.updateLeaf(node, key, nodeJustCreated);
               return node;
            }
            // child does not exist, but maybe it's a pruned node?
            if (!node.hasAtLeastOneChild() && !nodeJustCreated)
            { // current node does not have children AND it is not a new node -> expand pruned node
               expandNode(node, depth);
            }
            else
            { // not a pruned node, create requested child
               createNodeChild(node, childIndex, depth + 1);
               createdNode = true;
            }
         }

         NODE nodeChild = node.getChild(childIndex);

         if (updateRule.performLazyUpdate())
         {
            return updateNodeRecursively(nodeChild, createdNode, key, updateRule, depth + 1);
         }
         else
         {
            NODE leafToReturn = updateNodeRecursively(nodeChild, createdNode, key, updateRule, depth + 1);

            // That's an inner node, apply the update rule
            updateRule.updateInnerNode(node);

            // prune node if possible, otherwise set own probability
            // note: combining both did not lead to a speedup!
            if (updateRule.deleteUpdatedNode(leafToReturn))
            {
               deleteNodeChild(node, childIndex);

               // Update the parent, properties changed.
               updateRule.updateInnerNode(node);

               leafToReturn = node;
            }
            else if (pruneNode(node)) // return pointer to current parent (pruned), the just updated node no longer exists
            {
               leafToReturn = node;
            }

            return leafToReturn;
         }
      }
      else // at last level, update node, end of recursion
      {
         updateRule.updateLeaf(node, key, nodeJustCreated);
         return node;
      }
   }

   /// recursive call of prune()
   private int pruneRecursively(NODE node, int depth, int maxDepth, int numberOfPrunedNode)
   {
      if (node == null)
         throw new RuntimeException("The given node is null");

      if (!node.hasAtLeastOneChild())
         return numberOfPrunedNode;

      if (depth < maxDepth)
      {
         for (int i = 0; i < 8; i++)
         {
            NODE childNode = node.getChild(i);
            if (childNode != null)
               numberOfPrunedNode = pruneRecursively(childNode, depth + 1, maxDepth, numberOfPrunedNode);
         }
      } // end if depth
      else
      {
         // max level reached
         if (pruneNode(node))
         {
            numberOfPrunedNode++;
         }
      }

      return numberOfPrunedNode;
   }

   /** recursive call of expand() */
   private void expandRecursively(NODE node, int depth, int maxDepth)
   {
      if (depth >= maxDepth)
         return;

      if (node == null)
         throw new RuntimeException("The given node is null");

      // current node has no children => can be expanded
      if (!node.hasAtLeastOneChild())
      {
         expandNode(node, depth);
      }
      // recursively expand children
      for (int i = 0; i < 8; i++)
      {
         NODE childNode = node.getChild(i);
         if (childNode != null)
            expandRecursively(childNode, depth + 1, maxDepth);
      }
   }

   protected abstract Class<NODE> getNodeClass();
}
