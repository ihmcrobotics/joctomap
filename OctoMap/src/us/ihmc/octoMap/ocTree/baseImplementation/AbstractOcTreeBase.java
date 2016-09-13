package us.ihmc.octoMap.ocTree.baseImplementation;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.iterators.LeafBoundingBoxIterable;
import us.ihmc.octoMap.iterators.LeafIterable;
import us.ihmc.octoMap.iterators.OcTreeIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.node.NodeBuilder;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.ocTree.rules.interfaces.EarlyAbortRule;
import us.ihmc.octoMap.ocTree.rules.interfaces.UpdateRule;
import us.ihmc.octoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.octoMap.tools.OcTreeKeyTools;
import us.ihmc.octoMap.tools.OcTreeSearchTools;
import us.ihmc.octoMap.tools.OctoMapTools;

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
public abstract class AbstractOcTreeBase<NODE extends AbstractOcTreeNode<NODE>> implements Iterable<OcTreeSuperNode<NODE>>
{
   protected NODE root; ///< root NODE, null for empty tree
   private final NodeBuilder<NODE> nodeBuilder;
   private final List<NODE> unusedNodes = new ArrayList<>(50000000);
   private final List<NODE[]> unusedNodeArrays = new ArrayList<>(50000000 / 8);

   // constants of the tree
   /** Maximum tree depth (fixed to 16 usually) */
   protected final int treeDepth;
   protected double resolution; ///< in meters

   protected int treeSize; ///< number of nodes in tree
   /** flag to denote whether the octree extent changed (for lazy min/max eval) */
   protected boolean sizeChanged;

   protected double maxCoordinate[] = new double[3]; ///< max in x, y, z
   protected double minCoordinate[] = new double[3]; ///< min in x, y, z
   protected boolean lazyUpdate = false;

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
      this.treeDepth = treeDepth;
      treeSize = 0;
      nodeBuilder = new NodeBuilder<>(getNodeClass());

      initialize();
      // no longer create an empty root node - only on demand
      ensureCapacityUnusedPools(2000000);
   }

   public AbstractOcTreeBase(AbstractOcTreeBase<NODE> other)
   {
      resolution = other.resolution;
      treeDepth = other.treeDepth;
      nodeBuilder = new NodeBuilder<>(getNodeClass());
      initialize();
      if (other.root != null)
         root = other.root.cloneRecursive(nodeBuilder);
   }

   @SuppressWarnings("unchecked")
   public void ensureCapacityUnusedPools(int minCapacity)
   {
      while (unusedNodes.size() < minCapacity)
         unusedNodes.add(nodeBuilder.createNode());

      while (unusedNodeArrays.size() < minCapacity)
         unusedNodeArrays.add((NODE[]) Array.newInstance(getNodeClass(), 8));
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
   public boolean epsilonEquals(AbstractOcTreeBase<NODE> other)
   {
      if (treeDepth != other.treeDepth || resolution != other.resolution || treeSize != other.treeSize)
      {
         return false;
      }

      // traverse all nodes, check if structure the same
      Iterator<OcTreeSuperNode<NODE>> thisIterator = treeIterator();
      Iterator<OcTreeSuperNode<NODE>> otherIterator = other.treeIterator();

      for (OcTreeSuperNode<NODE> thisNode = thisIterator.next(), otherNode = otherIterator.next(); thisIterator.hasNext(); thisNode = thisIterator.next(), otherNode = otherIterator.next())
      {
         if (!otherIterator.hasNext()) // The other tree has less nodes
            return false;
         if (!thisNode.epsilonEquals(otherNode))
            return false;
      }

      if (otherIterator.hasNext()) // The other tree has more nodes
         return false;

      return true;
   }

   public final String getTreeType()
   {
      return getClass().getSimpleName();
   }

   /**
    * Affect the methods using {@link #updateNodeInternal(OcTreeKeyReadOnly, UpdateRule, EarlyAbortRule)}.
    * @param lazyUpdate whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    */
   public void setLazyUpdate(boolean lazyUpdate)
   {
      this.lazyUpdate = lazyUpdate;
   }

   /// Change the resolution of the octree, scaling all voxels.
   /// This will not preserve the (metric) scale!
   public void setResolution(double newResolution)
   {
      resolution = newResolution;
      sizeChanged = true;
   }

   public double getResolution()
   {
      return resolution;
   }

   public int getTreeDepth()
   {
      return treeDepth;
   }

   public double getNodeSize(int depth)
   {
      return OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
   }

   // -- Tree structure operations formerly contained in the nodes ---

   /// Creates (allocates) the i-th child of the node. @return ptr to newly create NODE
   protected NODE createNodeChild(NODE node, int childIndex)
   {
      OcTreeNodeTools.checkChildIndex(childIndex);
      assignChildrenArrayIfNecessary(node);

      if (node.getChildUnsafe(childIndex) != null)
         throw new RuntimeException("Something went wrong.");

      NODE newChildNode = getOrCreateNode();
      node.setChildUnsafe(childIndex, newChildNode);

      treeSize++;
      sizeChanged = true;

      return newChildNode;
   }

   private void assignChildrenArrayIfNecessary(NODE node)
   {
      if (!node.hasArrayForChildren())
      {
         if (unusedNodeArrays.isEmpty())
            node.allocateChildren();
         else
            node.assignChildren(unusedNodeArrays.remove(unusedNodeArrays.size() - 1));
      }
   }

   private NODE getOrCreateNode()
   {
      return unusedNodes.isEmpty() ? nodeBuilder.createNode() : unusedNodes.remove(unusedNodes.size() - 1);
   }

   /// Deletes the i-th child of the node
   public void deleteNodeChild(NODE node, int childIndex)
   {
      OcTreeNodeTools.checkChildIndex(childIndex);
      OcTreeNodeTools.checkNodeHasChildren(node);
      OcTreeNodeTools.checkNodeChildNotNull(node, childIndex);

      unusedNodes.add(node.removeChild(childIndex));

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
         root = getOrCreateNode();
         treeSize++;
         sizeChanged = true;
         createdRoot = true;
      }

      if (earlyAbortRule != null)
      {
         NODE leaf = OcTreeSearchTools.search(root, key, treeDepth);

         if (leaf != null)
         {
            if (earlyAbortRule.shouldAbortFullDepthUpdate(leaf))
               return leaf;
         }
      }

      return updateNodeRecurs(root, createdRoot, key, updateRule, 0);
   }

   /**
    * Expands a node (reverse of pruning): All children are created and
    * their occupancy probability is set to the node's value.
    *
    * You need to verify that this is indeed a pruned node (i.e. not a
    * leaf at the lowest level)
    *
    */
   public void expandNode(NODE node)
   {
      if (node.hasAtLeastOneChild())
         throw new RuntimeException("Node has already been expanded.");

      for (int k = 0; k < 8; k++)
      {
         NODE newNode = createNodeChild(node, k);
         newNode.copyData(node);
      }
   }

   /**
    * Prunes a node when it is collapsible
    * @return true if pruning was successful
    */
   public boolean pruneNode(NODE node)
   {
      if (!OcTreeNodeTools.isNodeCollapsible(node))
         return false;

      // set value to children's values (all assumed equal)
      node.copyData(node.getChildUnsafe(0));

      // delete children (known to be leafs at this point!)
      for (int childIndex = 0; childIndex < 8; childIndex++)
         deleteNodeChild(node, childIndex);
      unusedNodeArrays.add(node.removeChildren());

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
    *  Search node at specified depth given a 3d point (depth=0: search full tree depth).
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public NODE search(double x, double y, double z)
   {
      return OcTreeSearchTools.search(root, x, y, z, resolution, treeDepth);
   }

   public NODE search(double x, double y, double z, int depth)
   {
      return OcTreeSearchTools.search(root, x, y, z, depth, resolution, treeDepth);
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
    *  Delete a node (if exists) given a 3d point. Will always
    *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
    *  Pruned nodes at level "depth" will directly be deleted as a whole.
    */
   public boolean deleteNode(double x, double y, double z)
   {
      return deleteNode(x, y, z, 0);
   }

   public boolean deleteNode(double x, double y, double z, int depth)
   {
      OcTreeKey key = coordinateToKey(x, y, z);
      if (key == null)
      {
         System.err.println(AbstractOcTreeBase.class.getSimpleName() + " Error in deleteNode: [" + x + " " + y + " " + z + "] is out of OcTree bounds!");
         return false;
      }
      else
      {
         return deleteNode(key, depth);
      }
   }

   /** 
    *  Delete a node (if exists) given a 3d point. Will always
    *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
    *  Pruned nodes at level "depth" will directly be deleted as a whole.
    */
   public boolean deleteNode(Point3d value)
   {
      return deleteNode(value, 0);
   }

   public boolean deleteNode(Point3d coord, int depth)
   {
      return deleteNode(coord.getX(), coord.getY(), coord.getZ(), depth);
   }

   /** 
    *  Delete a node (if exists) given an addressing key. Will always
    *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
    *  Pruned nodes at level "depth" will directly be deleted as a whole.
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

   public double volume()
   {
      Vector3d metricSize = getMetricSize();
      return metricSize.getX() * metricSize.getY() * metricSize.getZ();
   }

   /// Size of OcTree (all known space) in meters for x, y and z dimension
   public Vector3d getMetricSize()
   {
      Vector3d size = new Vector3d();
      getMetricSize(size);
      return size;
   }

   public void getMetricSize(Vector3d size)
   {
      Point3d min = getMetricMin();
      Point3d max = getMetricMax();
      size.sub(max, min);
   }

   /// minimum value of the bounding box of all known space in x, y, z
   public Point3d getMetricMin()
   {
      Point3d min = new Point3d();
      getMetricMin(min);
      return min;
   }

   public void getMetricMin(Point3d min)
   {
      calculateMinMax();
      min.set(minCoordinate[0], minCoordinate[1], minCoordinate[2]);
   }

   /// maximum value of the bounding box of all known space in x, y, z
   public Point3d getMetricMax()
   {
      Point3d max = new Point3d();
      getMetricMax(max);
      return max;
   }

   public void getMetricMax(Point3d max)
   {
      calculateMinMax();
      max.set(maxCoordinate[0], maxCoordinate[1], maxCoordinate[2]);
   }

   /// Traverses the tree to calculate the total number of nodes
   public int calculateNumberOfNodes()
   {
      int retval = 0; // root node
      if (root != null)
      {
         retval = calculateNumberOfNodesRecursively(root, 1);
      }
      return retval;
   }

   /// Traverses the tree to calculate the total number of leaf nodes
   public int getNumLeafNodes()
   {
      if (root == null)
         return 0;

      return OcTreeNodeTools.getNumberOfLeafNodesRecursive(root);
   }

   // -- access tree nodes  ------------------

   /// return centers of leafs that do NOT exist (but could) in a given bounding box
   public void getUnknownLeafCenters(List<Point3d> nodeCenters, Point3d pmin, Point3d pmax)
   {
      getUnknownLeafCenters(nodeCenters, pmin, pmax, 0);
   }

   public void getUnknownLeafCenters(List<Point3d> nodeCenters, Point3d pmin, Point3d pmax, int depth)
   {
      OctoMapTools.checkIfDepthValid(depth, treeDepth);
      if (depth == 0)
         depth = treeDepth;

      double[] pminArray = new double[3];
      double[] pmaxArray = new double[3];
      pmin.get(pminArray);
      pmax.get(pmaxArray);

      double[] diff = new double[3];
      int[] steps = new int[3];
      double stepSize = resolution * Math.pow(2, treeDepth - depth);
      for (int i = 0; i < 3; ++i)
      {
         diff[i] = pmaxArray[i] - pminArray[i];
         steps[i] = (int) Math.floor(diff[i] / stepSize);
         //      std::cout << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
      }

      Point3d p = new Point3d(pmin);
      NODE res;
      for (int x = 0; x < steps[0]; ++x)
      {
         p.setX(p.getX() + stepSize);
         for (int y = 0; y < steps[1]; ++y)
         {
            p.setY(p.getY() + stepSize);
            for (int z = 0; z < steps[2]; ++z)
            {
               //          std::cout << "querying p=" << p << std::endl;
               p.setZ(p.getZ() + stepSize);
               res = search(p, depth);
               if (res == null)
               {
                  nodeCenters.add(p);
               }
            }
            p.setZ(pmin.getZ());
         }
         p.setY(pmin.getY());
      }
   }

   @Override
   public Iterator<OcTreeSuperNode<NODE>> iterator()
   {
      return leafIterable().iterator();
   }

   public Iterable<OcTreeSuperNode<NODE>> leafIterable()
   {
      return new LeafIterable<>(this);
   }

   public Iterable<OcTreeSuperNode<NODE>> leafIterable(int maxDepth)
   {
      return new LeafIterable<>(this, maxDepth);
   }

   public Iterator<OcTreeSuperNode<NODE>> treeIterator()
   {
      return treeIterable().iterator();
   }

   public Iterable<OcTreeSuperNode<NODE>> treeIterable()
   {
      return new OcTreeIterable<>(this);
   }

   public Iterable<OcTreeSuperNode<NODE>> treeIterable(int maxDepth)
   {
      return new OcTreeIterable<>(this, maxDepth);
   }

   public Iterable<OcTreeSuperNode<NODE>> leafBoundingBoxIterable(OcTreeKeyReadOnly min, OcTreeKeyReadOnly max)
   {
      LeafBoundingBoxIterable<NODE> iterable = new LeafBoundingBoxIterable<>(this, 0);
      iterable.setBoundingBox(min, max);
      return iterable; // TODO Organize imports;
   }

   //
   // Key / coordinate conversion functions
   //

   /** Converts from a single coordinate into a discrete key */
   public int coordinateToKey(double coordinate)
   {
      return OcTreeKeyConversionTools.coordinateToKey(coordinate, resolution, treeDepth);
   }

   /** Converts from a single coordinate into a discrete key at a given depth */
   public int coordinateToKey(double coordinate, int depth)
   {
      return OcTreeKeyConversionTools.coordinateToKey(coordinate, depth, coordinate, treeDepth);
   }

   /** Converts from a 3D coordinate into a 3D addressing key */
   public OcTreeKey coordinateToKey(Point3f coord)
   {
      return OcTreeKeyConversionTools.coordinateToKey(coord, resolution, treeDepth);
   }

   /** Converts from a 3D coordinate into a 3D addressing key */
   public boolean coordinateToKey(Point3f coord, OcTreeKey keyToPack)
   {
      return OcTreeKeyConversionTools.coordinateToKey(coord, resolution, treeDepth, keyToPack);
   }

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

   /// initialize non-trivial members, helper for constructors
   protected void initialize()
   {
      setResolution(resolution);
      for (int i = 0; i < 3; i++)
      {
         maxCoordinate[i] = Double.NEGATIVE_INFINITY;
         minCoordinate[i] = Double.POSITIVE_INFINITY;
      }
      sizeChanged = true;
   }

   /** Recalculates min and max in x, y, z. Does nothing when tree size didn't change. Reset {@link #sizeChanged} */
   protected void calculateMinMax()
   {
      if (!sizeChanged)
         return;

      // empty tree
      if (root == null)
      {
         minCoordinate[0] = minCoordinate[1] = minCoordinate[2] = 0.0;
         maxCoordinate[0] = maxCoordinate[1] = maxCoordinate[2] = 0.0;
         sizeChanged = false;
         return;
      }

      for (int i = 0; i < 3; i++)
      {
         maxCoordinate[i] = Double.NEGATIVE_INFINITY;
         minCoordinate[i] = Double.POSITIVE_INFINITY;
      }

      for (OcTreeSuperNode<NODE> node : this)
      {
         double size = node.getSize();
         double halfSize = size / 2.0;
         double x = node.getX() - halfSize;
         double y = node.getY() - halfSize;
         double z = node.getZ() - halfSize;
         if (x < minCoordinate[0])
            minCoordinate[0] = x;
         if (y < minCoordinate[1])
            minCoordinate[1] = y;
         if (z < minCoordinate[2])
            minCoordinate[2] = z;

         x += size;
         y += size;
         z += size;
         if (x > maxCoordinate[0])
            maxCoordinate[0] = x;
         if (y > maxCoordinate[1])
            maxCoordinate[1] = y;
         if (z > maxCoordinate[2])
            maxCoordinate[2] = z;
      }

      sizeChanged = false;
   }

   protected int calculateNumberOfNodesRecursively(NODE node, int currentNumberOfNodes)
   {
      if (node == null)
         throw new RuntimeException("The given node is null");

      if (node.hasAtLeastOneChild())
      {
         for (int i = 0; i < 8; i++)
         {
            if (OcTreeNodeTools.nodeChildExists(node, i))
            {
               currentNumberOfNodes++;
               currentNumberOfNodes = calculateNumberOfNodesRecursively(OcTreeNodeTools.getNodeChild(node, i), currentNumberOfNodes);
            }
         }
      }

      return currentNumberOfNodes;
   }

   /// recursive delete of node and all children (deallocates memory)
   protected void deleteNodeRecursively(NODE node)
   {
      if (node.hasAtLeastOneChild())
      {
         for (int i = 0; i < 8; i++)
         {
            NODE child = node.removeChildUnsafe(i);
            if (child != null)
            {
               unusedNodes.add(child);
               deleteNodeRecursively(child);
            }
         }

         unusedNodeArrays.add(node.removeChildren());
      } // else: node has no children
   }

   /// recursive call of deleteNode()
   protected boolean deleteNodeRecursively(NODE node, int depth, int maxDepth, OcTreeKeyReadOnly key)
   {
      if (depth >= maxDepth) // on last level: delete child when going up
         return true;

      if (node == null)
         throw new RuntimeException("The given node is null");

      int pos = OcTreeKeyTools.computeChildIndex(key, treeDepth - 1 - depth);

      if (!OcTreeNodeTools.nodeChildExists(node, pos))
      {
         // child does not exist, but maybe it's a pruned node?
         if (!node.hasAtLeastOneChild() && node != root)
         { // TODO double check for exists / root exception?
              // current node does not have children AND it's not the root node
           // -> expand pruned node
            expandNode(node);
            // tree_size and size_changed adjusted in createNodeChild(...)
         }
         else
         { // no branch here, node does not exist
            return false;
         }
      }

      // follow down further, fix inner nodes on way back up
      boolean deleteChild = deleteNodeRecursively(OcTreeNodeTools.getNodeChild(node, pos), depth + 1, maxDepth, key);
      if (deleteChild)
      {
         // TODO: lazy eval?
         // TODO delete check depth, what happens to inner nodes with children?
         deleteNodeChild(node, pos);

         if (!node.hasAtLeastOneChild())
            return true;
         else
         {
            node.updateOccupancyChildren(); // TODO: occupancy?
         }
      }
      // node did not lose a child, or still has other children
      return false;
   }

   private NODE updateNodeRecurs(NODE node, boolean nodeJustCreated, OcTreeKeyReadOnly key, UpdateRule<NODE> updateRule, int depth)
   {
      boolean createdNode = false;

      if (node == null)
         throw new RuntimeException("The given node is null.");

      // follow down to last level
      if (depth < treeDepth)
      {
         int pos = OcTreeKeyTools.computeChildIndex(key, treeDepth - 1 - depth);
         if (!OcTreeNodeTools.nodeChildExists(node, pos))
         {
            // child does not exist, but maybe it's a pruned node?
            if (!node.hasAtLeastOneChild() && !nodeJustCreated)
            { // current node does not have children AND it is not a new node -> expand pruned node
               expandNode(node);
            }
            else
            { // not a pruned node, create requested child
               createNodeChild(node, pos);
               createdNode = true;
            }
         }

         NODE nodeChild = OcTreeNodeTools.getNodeChild(node, pos);

         if (lazyUpdate)
         {
            return updateNodeRecurs(nodeChild, createdNode, key, updateRule, depth + 1);
         }
         else
         {
            NODE leafToReturn = updateNodeRecurs(nodeChild, createdNode, key, updateRule, depth + 1);
            // prune node if possible, otherwise set own probability
            // note: combining both did not lead to a speedup!
            if (pruneNode(node)) // return pointer to current parent (pruned), the just updated node no longer exists
               leafToReturn = node;
            else // That's an inner node, apply the update rule
               updateRule.updateInnerNode(node);

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
   protected int pruneRecursively(NODE node, int depth, int maxDepth, int numberOfPrunedNode)
   {
      if (node == null)
         throw new RuntimeException("The given node is null");

      if (!node.hasAtLeastOneChild())
         return numberOfPrunedNode;

      if (depth < maxDepth)
      {
         for (int i = 0; i < 8; i++)
         {
            NODE childNode;
            if ((childNode = node.getChildUnsafe(i)) != null)
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
   protected void expandRecursively(NODE node, int depth, int maxDepth)
   {
      if (depth >= maxDepth)
         return;

      if (node == null)
         throw new RuntimeException("The given node is null");

      // current node has no children => can be expanded
      if (!node.hasAtLeastOneChild())
      {
         expandNode(node);
      }
      // recursively expand children
      for (int i = 0; i < 8; i++)
      {
         if (OcTreeNodeTools.nodeChildExists(node, i))
         {
            expandRecursively(OcTreeNodeTools.getNodeChild(node, i), depth + 1, maxDepth);
         }
      }
   }

   protected abstract Class<NODE> getNodeClass();
}
