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
import us.ihmc.octoMap.node.OcTreeNodeTools;
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
   private List<NODE> unusedNodes = new ArrayList<>(50000000);
   private List<NODE[]> unusedNodeArrays = new ArrayList<>(50000000 / 8);

   // constants of the tree
   /** Maximum tree depth (fixed to 16 usually) */
   protected final int treeDepth;
   protected double resolution; ///< in meters

   protected int treeSize; ///< number of nodes in tree
   /** flag to denote whether the octree extent changed (for lazy min/max eval) */
   protected boolean size_changed;

   protected double max_value[] = new double[3]; ///< max in x, y, z
   protected double min_value[] = new double[3]; ///< min in x, y, z

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

      init();
      // no longer create an empty root node - only on demand
      ensureCapacityUnusedPools(2000000);
   }

   public AbstractOcTreeBase(AbstractOcTreeBase<NODE> other)
   {
      resolution = other.resolution;
      treeDepth = other.treeDepth;
      init();
      if (other.root != null)
         root = other.root.cloneRecursive();
   }

   @SuppressWarnings("unchecked")
   public void ensureCapacityUnusedPools(int minCapacity)
   {
      while (unusedNodes.size() < minCapacity)
         unusedNodes.add(createEmptyNode());
      
      NODE node = root != null ? root : createEmptyNode();
      while (unusedNodeArrays.size() < minCapacity)
         unusedNodeArrays.add((NODE[]) Array.newInstance(node.getClass(), 8));
   }

   /**
    * Swap contents of two octrees, i.e., only the underlying
    * pointer / tree structure. You have to ensure yourself that the
    * metadata (resolution etc) matches. No memory is cleared
    * in this function
    */
   public void swapContent(AbstractOcTreeBase<NODE> other)
   {
      NODE this_root = root;
      root = other.root;
      other.root = this_root;

      int this_size = treeSize;
      treeSize = other.treeSize;
      other.treeSize = this_size;
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

   /// Change the resolution of the octree, scaling all voxels.
   /// This will not preserve the (metric) scale!
   public void setResolution(double newResolution)
   {
      resolution = newResolution;
      size_changed = true;
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
   public NODE createNodeChild(NODE node, int childIndex)
   {
      OcTreeNodeTools.checkChildIndex(childIndex);
      if (!node.hasArrayForChildren())
      {
         if (unusedNodeArrays.isEmpty())
            node.allocateChildren();
         else
            node.assignChildren(unusedNodeArrays.remove(unusedNodeArrays.size() - 1));
      }

      if (node.getChildUnsafe(childIndex) != null)
         throw new RuntimeException("Something went wrong.");
      NODE newChildNode = unusedNodes.isEmpty() ? node.create() : unusedNodes.remove(unusedNodes.size() - 1);
      node.setChild(childIndex, newChildNode);

      treeSize++;
      size_changed = true;

      return newChildNode;
   }

   /// Deletes the i-th child of the node
   public void deleteNodeChild(NODE node, int childIndex)
   {
      OcTreeNodeTools.checkChildIndex(childIndex);
      OcTreeNodeTools.checkNodeHasChildren(node);
      OcTreeNodeTools.checkNodeChildNotNull(node, childIndex);

      unusedNodes.add(node.removeChild(childIndex));

      treeSize--;
      size_changed = true;
   }

   /**
    *  A node is collapsible if all children exist, don't have children of their own
    * and have the same occupancy value
    * @param node
    * @return
    */
   public boolean isNodeCollapsible(NODE node)
   {
//      // all children must exist, must not have children of
//      // their own and have the same occupancy probability
//      if (!OcTreeNodeTools.nodeChildExists(node, 0))
//         return false;
//
//      NODE firstChild = OcTreeNodeTools.getNodeChild(node, 0);
//      if (firstChild.hasAtLeastOneChild())
//         return false;
//
//      for (int i = 1; i < 8; i++)
//      {
//         if (!OcTreeNodeTools.nodeChildExists(node, i))
//            return false;
//
//         NODE currentChild = OcTreeNodeTools.getNodeChild(node, i);
//
//         if (currentChild.hasAtLeastOneChild() || !currentChild.epsilonEquals(firstChild))
//            return false;
//      }
//
//      return true;

      if (!node.hasArrayForChildren())
         return false;

      NODE firstChild = node.getChildUnsafe(0);
      if (firstChild == null || firstChild.hasAtLeastOneChild())
         return false;

      for (int i = 1; i < 8; i++)
      {
         NODE currentChild = node.getChildUnsafe(i);

         if (currentChild == null || currentChild.hasAtLeastOneChild() || !currentChild.epsilonEquals(firstChild))
            return false;
      }
      return true;
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
      if (!isNodeCollapsible(node))
         return false;

      // set value to children's values (all assumed equal)
      node.copyData(OcTreeNodeTools.getNodeChild(node, 0));

      // delete children (known to be leafs at this point!)
      for (int i = 0; i < 8; i++)
      {
         deleteNodeChild(node, i);
      }
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
      OcTreeKey key = coordinateToKey(coord);
      if (key == null)
      {
         System.err.println(AbstractOcTreeBase.class.getSimpleName() + " Error in deleteNode: [" + coord + "] is out of OcTree bounds!");
         return false;
      }
      else
      {
         return deleteNode(key, depth);
      }

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

      return deleteNodeRecurs(root, 0, depth, key);
   }

   /// Deletes the complete tree structure
   public void clear()
   {
      if (root != null)
      {
         deleteNodeRecurs(root);
         root = null;
         treeSize = 0;
         // max extent of tree changed:
         size_changed = true;
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
         int num_pruned = pruneRecurs(root, 0, depth, 0);
         if (num_pruned == 0)
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
         expandRecurs(root, 0, treeDepth);
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
      calcMinMax();
      min.set(min_value[0], min_value[1], min_value[2]);
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
      calcMinMax();
      max.set(max_value[0], max_value[1], max_value[2]);
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

      return getNumLeafNodesRecurs(root);
   }

   // -- access tree nodes  ------------------

   /// return centers of leafs that do NOT exist (but could) in a given bounding box
   public void getUnknownLeafCenters(List<Point3d> node_centers, Point3d pmin, Point3d pmax)
   {
      getUnknownLeafCenters(node_centers, pmin, pmax, 0);
   }

   public void getUnknownLeafCenters(List<Point3d> node_centers, Point3d pmin, Point3d pmax, int depth)
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
      double step_size = resolution * Math.pow(2, treeDepth - depth);
      for (int i = 0; i < 3; ++i)
      {
         diff[i] = pmaxArray[i] - pminArray[i];
         steps[i] = (int) Math.floor(diff[i] / step_size);
         //      std::cout << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
      }

      Point3d p = new Point3d(pmin);
      NODE res;
      for (int x = 0; x < steps[0]; ++x)
      {
         p.setX(p.getX() + step_size);
         for (int y = 0; y < steps[1]; ++y)
         {
            p.setY(p.getY() + step_size);
            for (int z = 0; z < steps[2]; ++z)
            {
               //          std::cout << "querying p=" << p << std::endl;
               p.setZ(p.getZ() + step_size);
               res = search(p, depth);
               if (res == null)
               {
                  node_centers.add(p);
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
      return OcTreeKeyConversionTools.convertCartesianCoordinateToKey(x, y, z, depth, resolution, treeDepth);
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
   protected void init()
   {
      setResolution(resolution);
      for (int i = 0; i < 3; i++)
      {
         max_value[i] = Double.NEGATIVE_INFINITY;
         min_value[i] = Double.POSITIVE_INFINITY;
      }
      size_changed = true;
   }

   /// recalculates min and max in x, y, z. Does nothing when tree size didn't change.
   protected void calcMinMax()
   {
      if (!size_changed)
         return;

      // empty tree
      if (root == null)
      {
         min_value[0] = min_value[1] = min_value[2] = 0.0;
         max_value[0] = max_value[1] = max_value[2] = 0.0;
         size_changed = false;
         return;
      }

      for (int i = 0; i < 3; i++)
      {
         max_value[i] = Double.NEGATIVE_INFINITY;
         min_value[i] = Double.POSITIVE_INFINITY;
      }

      for (OcTreeSuperNode<NODE> node : this)
      {
         double size = node.getSize();
         double halfSize = size / 2.0;
         double x = node.getX() - halfSize;
         double y = node.getY() - halfSize;
         double z = node.getZ() - halfSize;
         if (x < min_value[0])
            min_value[0] = x;
         if (y < min_value[1])
            min_value[1] = y;
         if (z < min_value[2])
            min_value[2] = z;

         x += size;
         y += size;
         z += size;
         if (x > max_value[0])
            max_value[0] = x;
         if (y > max_value[1])
            max_value[1] = y;
         if (z > max_value[2])
            max_value[2] = z;

      }

      size_changed = false;
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
   protected void deleteNodeRecurs(NODE node)
   {
      if (node == null)
         throw new RuntimeException("The given node is null");
      // TODO: maintain tree size?

      if (node.hasAtLeastOneChild())
      {
         for (int i = 0; i < 8; i++)
         {
            NODE child = node.removeChildUnsafe(i);
            if (child != null)
            {
               unusedNodes.add(child);
               deleteNodeRecurs(child);
            }
         }

         unusedNodeArrays.add(node.removeChildren());
      } // else: node has no children
   }

   /// recursive call of deleteNode()
   protected boolean deleteNodeRecurs(NODE node, int depth, int max_depth, OcTreeKeyReadOnly key)
   {
      if (depth >= max_depth) // on last level: delete child when going up
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
      boolean deleteChild = deleteNodeRecurs(OcTreeNodeTools.getNodeChild(node, pos), depth + 1, max_depth, key);
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

   /// recursive call of prune()
   protected int pruneRecurs(NODE node, int depth, int max_depth, int num_pruned)
   {
      if (node == null)
         throw new RuntimeException("The given node is null");

      if (depth < max_depth)
      {
         for (int i = 0; i < 8; i++)
         {
            if (OcTreeNodeTools.nodeChildExists(node, i))
            {
               num_pruned = pruneRecurs(OcTreeNodeTools.getNodeChild(node, i), depth + 1, max_depth, num_pruned);
            }
         }
      } // end if depth
      else
      {
         // max level reached
         if (pruneNode(node))
         {
            num_pruned++;
         }
      }

      return num_pruned;
   }

   /** recursive call of expand() */
   protected void expandRecurs(NODE node, int depth, int maxDepth)
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
            expandRecurs(OcTreeNodeTools.getNodeChild(node, i), depth + 1, maxDepth);
         }
      }
   }

   protected int getNumLeafNodesRecurs(NODE parent)
   {
      if (parent == null)
         throw new RuntimeException("The given parent node is null");

      if (!parent.hasAtLeastOneChild()) // this is a leaf -> terminate
         return 1;

      int sumLeafsChildren = 0;
      for (int i = 0; i < 8; ++i)
      {
         if (OcTreeNodeTools.nodeChildExists(parent, i))
         {
            sumLeafsChildren += getNumLeafNodesRecurs(OcTreeNodeTools.getNodeChild(parent, i));
         }
      }
      return sumLeafsChildren;
   }

   protected abstract NODE createEmptyNode();
}
