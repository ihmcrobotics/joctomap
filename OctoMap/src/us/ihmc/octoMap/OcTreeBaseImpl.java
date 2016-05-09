package us.ihmc.octoMap;

import static us.ihmc.octoMap.OcTreeKey.computeChildIdx;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.OcTreeIterator.LeafBoundingBoxIterator;
import us.ihmc.octoMap.OcTreeIterator.LeafIterator;
import us.ihmc.octoMap.OcTreeIterator.TreeIterator;
import us.ihmc.octoMap.OcTreeKey.KeyRay;
import us.ihmc.robotics.MathTools;
import us.ihmc.tools.io.printing.PrintTools;

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
 * \tparam INTERFACE Interface to be derived from, should be either
 *    AbstractOcTree or AbstractOccupancyOcTree
 */
public abstract class OcTreeBaseImpl<V, NODE extends OcTreeDataNode<V>>
{
   protected NODE root; ///< root NODE, null for empty tree

   // constants of the tree
   protected final int tree_depth; ///< Maximum tree depth is fixed to 16 currently
   protected final int tree_max_val;
   protected double resolution; ///< in meters
   protected double resolution_factor; ///< = 1. / resolution

   protected int tree_size; ///< number of nodes in tree
   /// flag to denote whether the octree extent changed (for lazy min/max eval)
   protected boolean size_changed;

   protected Point3d tree_center = new Point3d(); // coordinate offset of tree

   protected double max_value[] = new double[3]; ///< max in x, y, z
   protected double min_value[] = new double[3]; ///< min in x, y, z
   /// contains the size of a voxel at level i (0: root node). tree_depth+1 levels (incl. 0)
   protected List<Double> sizeLookupTable = new ArrayList<>();

   /// data structure for ray casting, array for multithreading
   protected List<KeyRay> keyrays = new ArrayList<>();

   protected final LeafIterator<V, NODE> leaf_iterator_end = new LeafIterator<>();
   protected final LeafBoundingBoxIterator<V, NODE> leaf_iterator_bbx_end = new LeafBoundingBoxIterator<>();
   protected final TreeIterator<V, NODE> tree_iterator_end = new TreeIterator<>();

   public OcTreeBaseImpl(double resolution)
   {
      this(resolution, 16, 32768);
   }

   /// Constructor to enable derived classes to change tree constants.
   /// This usually requires a re-implementation of some core tree-traversal functions as well!
   protected OcTreeBaseImpl(double resolution, int tree_depth, int tree_max_val)
   {
      root = null;
      this.resolution = resolution;
      this.tree_depth = tree_depth;
      this.tree_max_val = tree_max_val;
      tree_size = 0;

      init();
      // no longer create an empty root node - only on demand
   }

   @SuppressWarnings("unchecked")
   public OcTreeBaseImpl(OcTreeBaseImpl<V, NODE> other)
   {
      resolution = other.resolution;
      tree_depth = other.tree_depth;
      tree_max_val = other.tree_max_val;
      init();
      if (other.root != null)
         root = (NODE) other.root.clone();
   }

   /**
    * Swap contents of two octrees, i.e., only the underlying
    * pointer / tree structure. You have to ensure yourself that the
    * metadata (resolution etc) matches. No memory is cleared
    * in this function
    */
   public void swapContent(OcTreeBaseImpl<V, NODE> other)
   {
      NODE this_root = root;
      root = other.root;
      other.root = this_root;

      int this_size = tree_size;
      tree_size = other.tree_size;
      other.tree_size = this_size;
   }

   /// Comparison between two octrees, all meta data, all
   /// nodes, and the structure must be identical
   public boolean equals(OcTreeBaseImpl<V, NODE> other)
   {

      if (tree_depth != other.tree_depth || tree_max_val != other.tree_max_val || resolution != other.resolution || tree_size != other.tree_size)
      {
         return false;
      }

      // traverse all nodes, check if structure the same
      TreeIterator<V, NODE> it = begin_tree();
      TreeIterator<V, NODE> end = end_tree();
      TreeIterator<V, NODE> other_it = other.begin_tree();
      TreeIterator<V, NODE> other_end = other.end_tree();

      for (; !it.equals(end); it.next(), other_it.next())
      {
         if (other_it.equals(other_end))
            return false;

         if (it.getDepth() != other_it.getDepth() || !(it.getKey().equals(other_it.getKey())) || !(it.getNode().equals(other_it.getNode())))
         {
            return false;
         }
      }

      if (!(other_it.equals(other_end)))
         return false;

      return true;
   }

   public String getTreeType()
   {
      return "OcTreeBaseImpl";
   }

   /// Change the resolution of the octree, scaling all voxels.
   /// This will not preserve the (metric) scale!
   public void setResolution(double r)
   {
      resolution = r;
      resolution_factor = 1. / resolution;

      tree_center.x = tree_center.y = tree_center.z = (float) (((double) tree_max_val) / resolution_factor);

      // init node size lookup table:
      sizeLookupTable.clear();
      for (int i = 0; i <= tree_depth; ++i)
      {
         sizeLookupTable.add(resolution * (double) (1 << (tree_depth - i)));
      }

      size_changed = true;
   }

   public double getResolution()
   {
      return resolution;
   }

   public int getTreeDepth()
   {
      return tree_depth;
   }

   public double getNodeSize(int depth)
   {
      MathTools.checkIfLessOrEqual(depth, tree_depth);
      return sizeLookupTable.get(depth);
   }

   /**
    * Clear KeyRay vector to minimize unneeded memory. This is only
    * useful for the StaticMemberInitializer classes, don't call it for
    * an octree that is actually used.
    */
   public void clearKeyRays()
   {
      keyrays.clear();
   }

   // -- Tree structure operations formerly contained in the nodes ---

   /// Creates (allocates) the i-th child of the node. @return ptr to newly create NODE
   @SuppressWarnings("unchecked")
   public NODE createNodeChild(NODE node, int childIdx)
   {
      checkChildIndex(childIdx);
      if (node.children == null)
      {
         allocNodeChildren(node);
      }
      if (node.children[childIdx] != null)
         throw new RuntimeException("Something went wrong.");
      NODE newNode = (NODE) node.create();
      node.children[childIdx] = newNode;

      tree_size++;
      size_changed = true;

      return newNode;
   }

   public void checkChildIndex(int childIdx)
   {
      if (childIdx > 7)
         throw new RuntimeException("Bad child index :" + childIdx + ", expected index to be in [0, 7].");
   }

   /// Deletes the i-th child of the node
   public void deleteNodeChild(NODE node, int childIdx)
   {
      checkChildIndex(childIdx);
      checkNodeHasChildren(node);
      checkNodeChildNotNull(node, childIdx);

      node.children[childIdx] = null;

      tree_size--;
      size_changed = true;
   }

   public void checkNodeChildNotNull(NODE node, int childIdx)
   {
      if (node.children[childIdx] == null)
         throw new RuntimeException("Child is already null.");
   }

   public void checkNodeHasChildren(NODE node)
   {
      if (node.children == null)
         throw new RuntimeException("The given node has no children.");
   }

   /// @return ptr to child number childIdx of node
   @SuppressWarnings("unchecked")
   public NODE getNodeChild(NODE node, int childIdx)
   {
      checkChildIndex(childIdx);
      checkNodeHasChildren(node);
      checkNodeChildNotNull(node, childIdx);
      return (NODE) node.children[childIdx];
   }

   /// A node is collapsible if all children exist, don't have children of their own
   /// and have the same occupancy value
   public boolean isNodeCollapsible(NODE node)
   {
      // all children must exist, must not have children of
      // their own and have the same occupancy probability
      if (!nodeChildExists(node, 0))
         return false;

      NODE firstChild = getNodeChild(node, 0);
      if (nodeHasChildren(firstChild))
         return false;

      for (int i = 1; i < 8; i++)
      {
         // comparison via getChild so that casts of derived classes ensure
         // that the right == operator gets called
         NODE currentChild = getNodeChild(node, i);
         if (!nodeChildExists(node, i) || nodeHasChildren(currentChild) || !(currentChild.equals(firstChild)))
            return false;
      }

      return true;
   }

   /** 
    * Safe test if node has a child at index childIdx.
    * First tests if there are any children. Replaces node->childExists(...)
    * \return true if the child at childIdx exists
    */
   public boolean nodeChildExists(NODE node, int childIdx)
   {
      checkChildIndex(childIdx);
      if ((node.children != null) && (node.children[childIdx] != null))
         return true;
      else
         return false;
   }

   /** 
    * Safe test if node has any children. Replaces node->hasChildren(...)
    * \return true if node has at least one child
    */
   public boolean nodeHasChildren(NODE node)
   {
      if (node.children == null)
         return false;

      for (int i = 0; i < 8; i++)
      {
         if (node.children[i] != null)
            return true;
      }
      return false;
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
      if (nodeHasChildren(node))
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
      node.copyData(getNodeChild(node, 0));

      // delete children (known to be leafs at this point!)
      for (int i = 0; i < 8; i++)
      {
         deleteNodeChild(node, i);
      }
      node.children = null;

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
      return search(x, y, z, 0);
   }

   public NODE search(double x, double y, double z, int depth)
   {
      OcTreeKey key = new OcTreeKey();
      if (!coordToKeyChecked(x, y, z, key))
      {
         PrintTools.error(this, "Error in search: [" + x + " " + y + " " + z + "] is out of OcTree bounds!");
         return null;
      }
      else
      {
         return search(key, depth);
      }
   }

   /**
    *  Search node at specified depth given a 3d point (depth=0: search full tree depth)
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public NODE search(Point3d value)
   {
      return search(value, 0);
   }

   public NODE search(Point3d value, int depth)
   {
      OcTreeKey key = new OcTreeKey();
      if (!coordToKeyChecked(value, key))
      {
         PrintTools.error(this, "Error in search: [" + value + "] is out of OcTree bounds!");
         return null;
      }
      else
      {
         return search(key, depth);
      }

   }

   /**
    *  Search a node at specified depth given an addressing key (depth=0: search full tree depth)
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public NODE search(OcTreeKey key)
   {
      return search(key, 0);
   }

   public NODE search(OcTreeKey key, int depth)
   {
      MathTools.checkIfLessOrEqual(depth, tree_depth);
      if (root == null)
         return null;

      if (depth == 0)
         depth = tree_depth;

      // generate appropriate key_at_depth for queried depth
      OcTreeKey key_at_depth = new OcTreeKey(key);
      if (depth != tree_depth)
         key_at_depth = adjustKeyAtDepth(key, depth);

      NODE curNode = root;

      int diff = tree_depth - depth;

      // follow nodes down to requested level (for diff = 0 it's the last level)
      for (int i = (tree_depth - 1); i >= diff; --i)
      {
         int pos = computeChildIdx(key_at_depth, i);
         if (nodeChildExists(curNode, pos))
         {
            // cast needed: (nodes need to ensure it's the right pointer)
            curNode = getNodeChild(curNode, pos);
         }
         else
         {
            // we expected a child but did not get it
            // is the current node a leaf already?
            if (!nodeHasChildren(curNode))
            { // TODO similar check to nodeChildExists?
               return curNode;
            }
            else
            {
               // it is not, search failed
               return null;
            }
         }
      } // end for
      return curNode;
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
      OcTreeKey key = new OcTreeKey();
      if (!coordToKeyChecked(x, y, z, key))
      {
         PrintTools.error(this, "Error in deleteNode: [" + x + " " + y + " " + z + "] is out of OcTree bounds!");
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

   public boolean deleteNode(Point3d value, int depth)
   {
      OcTreeKey key = new OcTreeKey();
      if (!coordToKeyChecked(value, key))
      {
         PrintTools.error(this, "Error in deleteNode: [" + value + "] is out of OcTree bounds!");
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
   public boolean deleteNode(OcTreeKey key)
   {
      return deleteNode(key, 0);
   }

   public boolean deleteNode(OcTreeKey key, int depth)
   {
      if (root == null)
         return true;

      if (depth == 0)
         depth = tree_depth;

      return deleteNodeRecurs(root, 0, depth, key);
   }

   /// Deletes the complete tree structure
   public void clear()
   {
      if (root != null)
      {
         deleteNodeRecurs(root);
         tree_size = 0;
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

      for (int depth = tree_depth - 1; depth > 0; --depth)
      {
         int num_pruned = 0;
         pruneRecurs(root, 0, depth, num_pruned);
         if (num_pruned == 0)
            break;
      }
   }

   /// Expands all pruned nodes (reverse of prune())
   /// \note This is an expensive operation, especially when the tree is nearly empty!
   public void expand()
   {
      if (root != null)
         expandRecurs(root, 0, tree_depth);
   }

   // -- statistics  ----------------------

   /// \return The number of nodes in the tree
   public int size()
   {
      return tree_size;
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
   public int calcNumNodes()
   {
      int retval = 0; // root node
      if (root != null)
      {
         retval++;
         calcNumNodesRecurs(root, retval);
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

      MathTools.checkIfLessOrEqual(depth, tree_depth);
      if (depth == 0)
         depth = tree_depth;

      double[] pminArray = new double[3];
      double[] pmaxArray = new double[3];
      pmin.get(pminArray);
      pmax.get(pmaxArray);

      double[] diff = new double[3];
      int[] steps = new int[3];
      double step_size = resolution * Math.pow(2, tree_depth - depth);
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

   // -- raytracing  -----------------------

   /**
   * Traces a ray from origin to end (excluding), returning an
   * OcTreeKey of all nodes traversed by the beam. You still need to check
   * if a node at that coordinate exists (e.g. with search()).
   *
   * @param origin start coordinate of ray
   * @param end end coordinate of ray
   * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
   * @return Success of operation. Returning false usually means that one of the coordinates is out of the OcTree's range
   */
   public boolean computeRayKeys(Point3d origin, Point3d end, KeyRay ray)
   {

      // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
      // basically: DDA in 3D

      ray.reset();

      OcTreeKey key_origin = new OcTreeKey();
      OcTreeKey key_end = new OcTreeKey();
      if (!coordToKeyChecked(origin, key_origin) || !coordToKeyChecked(end, key_end))
      {
         PrintTools.error(this, "coordinates ( " + origin + " -> " + end + ") out of bounds in computeRayKeys");
         return false;
      }

      if (key_origin.equals(key_end))
         return true; // same tree cell, we're done.

      ray.addKey(key_origin);

      // Initialization phase -------------------------------------------------------

      Vector3d direction = new Vector3d();
      direction.sub(end, origin);
      double length = direction.length();
      direction.scale(1.0 / length);

      double[] directionArray = new double[3];
      double[] originArray = new double[3];

      direction.get(directionArray);
      origin.get(originArray);

      int[] step = new int[3];
      double[] tMax = new double[3];
      double[] tDelta = new double[3];

      OcTreeKey current_key = new OcTreeKey(key_origin);

      for (int i = 0; i < 3; ++i)
      {
         // compute step direction
         if (directionArray[i] > 0.0)
            step[i] = 1;
         else if (directionArray[i] < 0.0)
            step[i] = -1;
         else
            step[i] = 0;

         // compute tMax, tDelta
         if (step[i] != 0)
         {
            // corner point of voxel (in direction of ray)
            double voxelBorder = keyToCoord(current_key.k[i]);
            voxelBorder += (float) (step[i] * resolution * 0.5);

            tMax[i] = (voxelBorder - originArray[i]) / directionArray[i];
            tDelta[i] = resolution / Math.abs(directionArray[i]);
         }
         else
         {
            tMax[i] = Double.MAX_VALUE;
            tDelta[i] = Double.MAX_VALUE;
         }
      }

      // Incremental phase  ---------------------------------------------------------

      boolean done = false;
      while (!done)
      {

         int dim;

         // find minimum tMax:
         if (tMax[0] < tMax[1])
         {
            if (tMax[0] < tMax[2])
               dim = 0;
            else
               dim = 2;
         }
         else
         {
            if (tMax[1] < tMax[2])
               dim = 1;
            else
               dim = 2;
         }

         // advance in direction "dim"
         current_key.k[dim] += step[dim];
         tMax[dim] += tDelta[dim];

         if (current_key.k[dim] >= 2 * tree_max_val)
            throw new RuntimeException("Something went wrong.");

         // reached endpoint, key equv?
         if (current_key.equals(key_end))
         {
            done = true;
            break;
         }
         else
         {

            // reached endpoint world coords?
            // dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
            double dist_from_origin = Math.min(Math.min(tMax[0], tMax[1]), tMax[2]);
            // if this is longer than the expected ray length, we should have already hit the voxel containing the end point with the code above (key_end).
            // However, we did not hit it due to accumulating discretization errors, so this is the point here to stop the ray as we would never reach the voxel key_end
            if (dist_from_origin > length)
            {
               done = true;
               break;
            }

            else
            { // continue to add freespace cells
               ray.addKey(current_key);
            }
         }

         assert (ray.size() < ray.sizeMax() - 1);

      } // end while

      return true;
   }

   /**
   * Traces a ray from origin to end (excluding), returning the
   * coordinates of all nodes traversed by the beam. You still need to check
   * if a node at that coordinate exists (e.g. with search()).
   * @note: use the faster computeRayKeys method if possible.
   * 
   * @param origin start coordinate of ray
   * @param end end coordinate of ray
   * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
   * @return Success of operation. Returning false usually means that one of the coordinates is out of the OcTree's range
   */
   public boolean computeRay(Point3d origin, Point3d end, List<Point3d> ray)
   {
      ray.clear();

      if (!computeRayKeys(origin, end, keyrays.get(0)))
         return false;
      for (OcTreeKey key : keyrays.get(0))
      {
         ray.add(keyToCoord(key));
      }
      return true;
   }

   /// @return beginning of the tree as leaf iterator
   public LeafIterator<V, NODE> begin()
   {
      return begin(0);
   }

   public LeafIterator<V, NODE> begin(int maxDepth)
   {
      return new LeafIterator<>(this, maxDepth);
   };

   /// @return end of the tree as leaf iterator
   public LeafIterator<V, NODE> end()
   {
      return leaf_iterator_end;
   }; // TODO: RVE?

   /// @return beginning of the tree as leaf iterator
   public LeafIterator<V, NODE> begin_leafs()
   {
      return begin_leafs(0);
   };

   public LeafIterator<V, NODE> begin_leafs(int maxDepth)
   {
      return new LeafIterator<>(this, maxDepth);
   };

   /// @return end of the tree as leaf iterator
   public LeafIterator<V, NODE> end_leafs()
   {
      return leaf_iterator_end;
   }

   /// @return beginning of the tree as leaf iterator in a bounding box
   public LeafBoundingBoxIterator<V, NODE> begin_leafs_bbx(OcTreeKey min, OcTreeKey max)
   {
      return begin_leafs_bbx(min, max, 0);
   }

   public LeafBoundingBoxIterator<V, NODE> begin_leafs_bbx(OcTreeKey min, OcTreeKey max, int maxDepth)
   {
      return new LeafBoundingBoxIterator<>(this, min, max, maxDepth);
   }

   /// @return beginning of the tree as leaf iterator in a bounding box
   public LeafBoundingBoxIterator<V, NODE> begin_leafs_bbx(Point3d min, Point3d max)
   {
      return begin_leafs_bbx(min, max, 0);
   }

   public LeafBoundingBoxIterator<V, NODE> begin_leafs_bbx(Point3d min, Point3d max, int maxDepth)
   {
      return new LeafBoundingBoxIterator<>(this, min, max, maxDepth);
   }

   /// @return end of the tree as leaf iterator in a bounding box
   public LeafBoundingBoxIterator<V, NODE> end_leafs_bbx()
   {
      return leaf_iterator_bbx_end;
   }

   /// @return beginning of the tree as iterator to all nodes (incl. inner)
   public TreeIterator<V, NODE> begin_tree()
   {
      return begin_tree(0);
   }

   public TreeIterator<V, NODE> begin_tree(int maxDepth)
   {
      return new TreeIterator<>(this, maxDepth);
   }

   /// @return end of the tree as iterator to all nodes (incl. inner)
   public TreeIterator<V, NODE> end_tree()
   {
      return tree_iterator_end;
   }

   //
   // Key / coordinate conversion functions
   //

   /// Converts from a single coordinate into a discrete key
   public int coordToKey(double coordinate)
   {
      return (int) Math.floor(resolution_factor * coordinate) + tree_max_val;
   }

   /// Converts from a single coordinate into a discrete key at a given depth
   public int coordToKey(double coordinate, int depth)
   {
      MathTools.checkIfLessOrEqual(depth, tree_depth);
      int keyval = ((int) Math.floor(resolution_factor * coordinate));

      int diff = tree_depth - depth;
      if (diff != 0) // same as coordToKey without depth
         return keyval + tree_max_val;
      else // shift right and left => erase last bits. Then add offset.
         return ((keyval >> diff) << diff) + (1 << (diff - 1)) + tree_max_val;
   }

   /// Converts from a 3D coordinate into a 3D addressing key
   public OcTreeKey coordToKey(Point3d coord)
   {
      return new OcTreeKey(coordToKey(coord.x), coordToKey(coord.y), coordToKey(coord.z));
   }

   /// Converts from a 3D coordinate into a 3D addressing key
   public OcTreeKey coordToKey(double x, double y, double z)
   {
      return new OcTreeKey(coordToKey(x), coordToKey(y), coordToKey(z));
   }

   /// Converts from a 3D coordinate into a 3D addressing key at a given depth
   public OcTreeKey coordToKey(Point3d coord, int depth)
   {
      if (depth == tree_depth)
         return coordToKey(coord);
      else
         return new OcTreeKey(coordToKey(coord.x, depth), coordToKey(coord.y, depth), coordToKey(coord.z, depth));
   }

   /// Converts from a 3D coordinate into a 3D addressing key at a given depth
   public OcTreeKey coordToKey(double x, double y, double z, int depth)
   {
      if (depth == tree_depth)
         return coordToKey(x, y, z);
      else
         return new OcTreeKey(coordToKey(x, depth), coordToKey(y, depth), coordToKey(z, depth));
   }

   /**
    * Adjusts a 3D key from the lowest level to correspond to a higher depth (by
    * shifting the key values)
    *
    * @param key Input key, at the lowest tree level
    * @param depth Target depth level for the new key
    * @return Key for the new depth level
    */
   public OcTreeKey adjustKeyAtDepth(OcTreeKey key, int depth)
   {
      if (depth == tree_depth)
         return key;

      assert (depth <= tree_depth);
      return new OcTreeKey(adjustKeyAtDepth(key.k[0], depth), adjustKeyAtDepth(key.k[1], depth), adjustKeyAtDepth(key.k[2], depth));
   }

   /**
    * Adjusts a single key value from the lowest level to correspond to a higher depth (by
    * shifting the key value)
    *
    * @param key Input key, at the lowest tree level
    * @param depth Target depth level for the new key
    * @return Key for the new depth level
    */
   public int adjustKeyAtDepth(int key, int depth)
   {
      int diff = tree_depth - depth;

      if (diff == 0)
         return key;
      else
         return (((key - tree_max_val) >> diff) << diff) + (1 << (diff - 1)) + tree_max_val;
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @param key values that will be computed, an array of fixed size 3.
    * @return true if point is within the octree (valid), false otherwise
    */
   public boolean coordToKeyChecked(Point3d coord, OcTreeKey key)
   {
      key.k[0] = coordToKeyChecked(coord.x);
      key.k[1] = coordToKeyChecked(coord.y);
      key.k[2] = coordToKeyChecked(coord.z);
      return key.k[0] != -1 && key.k[1] != -1 && key.k[2] != -1;
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey at a certain depth, with boundary checking.
    *
    * @param coord 3d coordinate of a point
    * @param depth level of the key from the top
    * @param key values that will be computed, an array of fixed size 3.
    * @return true if point is within the octree (valid), false otherwise
    */
   public boolean coordToKeyChecked(Point3d coord, int depth, OcTreeKey key)
   {
      key.k[0] = coordToKeyChecked(coord.x, depth);
      key.k[1] = coordToKeyChecked(coord.y, depth);
      key.k[2] = coordToKeyChecked(coord.z, depth);
      return key.k[0] != -1 && key.k[1] != -1 && key.k[2] != -1;
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
    *
    * @param x
    * @param y
    * @param z
    * @param key values that will be computed, an array of fixed size 3.
    * @return true if point is within the octree (valid), false otherwise
    */
   public boolean coordToKeyChecked(double x, double y, double z, OcTreeKey key)
   {
      key.k[0] = coordToKeyChecked(x);
      key.k[1] = coordToKeyChecked(y);
      key.k[2] = coordToKeyChecked(z);
      return key.k[0] != -1 && key.k[1] != -1 && key.k[2] != -1;
   }

   /**
    * Converts a 3D coordinate into a 3D OcTreeKey at a certain depth, with boundary checking.
    *
    * @param x
    * @param y
    * @param z
    * @param depth level of the key from the top
    * @param key values that will be computed, an array of fixed size 3.
    * @return true if point is within the octree (valid), false otherwise
    */
   public boolean coordToKeyChecked(double x, double y, double z, int depth, OcTreeKey key)
   {
      key.k[0] = coordToKeyChecked(x, depth);
      key.k[1] = coordToKeyChecked(y, depth);
      key.k[2] = coordToKeyChecked(z, depth);
      return key.k[0] != -1 && key.k[1] != -1 && key.k[2] != -1;
   }

   /**
    * Converts a single coordinate into a discrete addressing key, with boundary checking.
    *
    * @param coordinate 3d coordinate of a point
    * @param key discrete 16 bit adressing key, result
    * @return true if coordinate is within the octree bounds (valid), false otherwise
    */
   public int coordToKeyChecked(double coordinate)
   {
      // scale to resolution and shift center for tree_max_val
      int scaled_coord = ((int) Math.floor(resolution_factor * coordinate)) + tree_max_val;

      // keyval within range of tree?
      if ((scaled_coord >= 0) && (((int) scaled_coord) < (2 * tree_max_val)))
      {
         return scaled_coord;
      }
      return -1;
   }

   /**
    * Converts a single coordinate into a discrete addressing key, with boundary checking.
    *
    * @param coordinate 3d coordinate of a point
    * @param depth level of the key from the top
    * @param key discrete 16 bit adressing key, result
    * @return true if coordinate is within the octree bounds (valid), false otherwise
    */
   public int coordToKeyChecked(double coordinate, int depth)
   {

      // scale to resolution and shift center for tree_max_val
      int scaled_coord = ((int) Math.floor(resolution_factor * coordinate)) + tree_max_val;

      // keyval within range of tree?
      if ((scaled_coord >= 0) && (((int) scaled_coord) < (2 * tree_max_val)))
      {
         int keyval = scaled_coord;
         keyval = adjustKeyAtDepth(keyval, depth);
         return keyval;
      }
      return -1;
   }

   /// converts from a discrete key at a given depth into a coordinate
   /// corresponding to the key's center
   public double keyToCoord(int key, int depth)
   {
      MathTools.checkIfLessOrEqual(depth, tree_depth);

      // root is centered on 0 = 0.0
      if (depth == 0)
      {
         return 0.0;
      }
      else if (depth == tree_depth)
      {
         return keyToCoord(key);
      }
      else
      {
         return (Math.floor(((double) (key) - (double) (tree_max_val)) / (double) (1 << (tree_depth - depth))) + 0.5) * getNodeSize(depth);
      }
   }

   /// converts from a discrete key at the lowest tree level into a coordinate
   /// corresponding to the key's center
   public double keyToCoord(int key)
   {
      return ((double) (key - tree_max_val) + 0.5) * resolution;
   }

   /// converts from an addressing key at the lowest tree level into a coordinate
   /// corresponding to the key's center
   public Point3d keyToCoord(OcTreeKey key)
   {
      return new Point3d(keyToCoord(key.k[0]), keyToCoord(key.k[1]), keyToCoord(key.k[2]));
   }

   /// converts from an addressing key at a given depth into a coordinate
   /// corresponding to the key's center
   public Point3d keyToCoord(OcTreeKey key, int depth)
   {
      return new Point3d(keyToCoord(key.k[0], depth), keyToCoord(key.k[1], depth), keyToCoord(key.k[2], depth));
   }

   /// initialize non-trivial members, helper for constructors
   protected void init()
   {
      setResolution(resolution);
      for (int i = 0; i < 3; i++)
      {
         max_value[i] = -Double.MAX_VALUE;
         min_value[i] = Double.MAX_VALUE;
      }
      size_changed = true;

      keyrays.add(new KeyRay());
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
         max_value[i] = -Double.MAX_VALUE;
         min_value[i] = Double.MAX_VALUE;
      }

      for (LeafIterator<V, NODE> it = begin(), end = end(); !(it.equals(end)); it.next())
      {
         double size = it.getSize();
         double halfSize = size / 2.0;
         double x = it.getX() - halfSize;
         double y = it.getY() - halfSize;
         double z = it.getZ() - halfSize;
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

   protected void calcNumNodesRecurs(NODE node, int num_nodes)
   {
      if (node == null)
         throw new RuntimeException("The given node is null");
      if (nodeHasChildren(node))
      {
         for (int i = 0; i < 8; ++i)
         {
            if (nodeChildExists(node, i))
            {
               num_nodes++;
               calcNumNodesRecurs(getNodeChild(node, i), num_nodes);
            }
         }
      }
   }

   /// recursive delete of node and all children (deallocates memory)
   @SuppressWarnings("unchecked")
   protected void deleteNodeRecurs(NODE node)
   {
      if (node == null)
         throw new RuntimeException("The given node is null");
      // TODO: maintain tree size?

      if (node.children != null)
      {
         for (int i = 0; i < 8; i++)
         {
            if (node.children[i] != null)
               deleteNodeRecurs((NODE) node.children[i]);
         }
         node.children = null;
      } // else: node has no children

      node = null;
   }

   /// recursive call of deleteNode()
   protected boolean deleteNodeRecurs(NODE node, int depth, int max_depth, OcTreeKey key)
   {
      if (depth >= max_depth) // on last level: delete child when going up
         return true;

      if (node == null)
         throw new RuntimeException("The given node is null");

      int pos = computeChildIdx(key, tree_depth - 1 - depth);

      if (!nodeChildExists(node, pos))
      {
         // child does not exist, but maybe it's a pruned node?
         if ((!nodeHasChildren(node)) && (node != root))
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
      boolean deleteChild = deleteNodeRecurs(getNodeChild(node, pos), depth + 1, max_depth, key);
      if (deleteChild)
      {
         // TODO: lazy eval?
         // TODO delete check depth, what happens to inner nodes with children?
         deleteNodeChild(node, pos);

         if (!nodeHasChildren(node))
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
   protected void pruneRecurs(NODE node, int depth, int max_depth, int num_pruned)
   {

      if (node == null)
         throw new RuntimeException("The given node is null");

      if (depth < max_depth)
      {
         for (int i = 0; i < 8; i++)
         {
            if (nodeChildExists(node, i))
            {
               pruneRecurs(getNodeChild(node, i), depth + 1, max_depth, num_pruned);
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
   }

   /// recursive call of expand()
   protected void expandRecurs(NODE node, int depth, int max_depth)
   {
      if (depth >= max_depth)
         return;

      if (node == null)
         throw new RuntimeException("The given node is null");

      // current node has no children => can be expanded
      if (!nodeHasChildren(node))
      {
         expandNode(node);
      }
      // recursively expand children
      for (int i = 0; i < 8; i++)
      {
         if (nodeChildExists(node, i))
         { // TODO double check (node != NULL)
            expandRecurs(getNodeChild(node, i), depth + 1, max_depth);
         }
      }
   }

   protected int getNumLeafNodesRecurs(NODE parent)
   {
      if (parent == null)
         throw new RuntimeException("The given parent node is null");

      if (!nodeHasChildren(parent)) // this is a leaf -> terminate
         return 1;

      int sum_leafs_children = 0;
      for (int i = 0; i < 8; ++i)
      {
         if (nodeChildExists(parent, i))
         {
            sum_leafs_children += getNumLeafNodesRecurs(getNodeChild(parent, i));
         }
      }
      return sum_leafs_children;
   }

   protected void allocNodeChildren(NODE node)
   {
      // TODO NODE*
      node.allocateChildren();
   }
}
