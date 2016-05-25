package us.ihmc.octoMap;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.OcTreeKey.KeyRay;
import us.ihmc.octoMap.iterators.LeafBoundingBoxIterable;
import us.ihmc.octoMap.iterators.LeafIterable;
import us.ihmc.octoMap.iterators.OcTreeIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.node.OcTreeDataNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.tools.OcTreeCoordinateConversionTools;
import us.ihmc.octoMap.tools.OcTreeKeyTools;
import us.ihmc.octoMap.tools.OcTreeSearchTools;
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
public abstract class OcTreeBaseImpl<NODE extends OcTreeDataNode<NODE>> implements Iterable<OcTreeSuperNode<NODE>>
{
   protected NODE root; ///< root NODE, null for empty tree

   // constants of the tree
   /** Maximum tree depth (fixed to 16 usually) */
   protected final int treeDepth;
   /** No documentation on this field, I assume it seems to be equal to 2^({@link #treeDepth}-1). */
   protected final int treeMaximumValue;
   protected double resolution; ///< in meters

   protected int treeSize; ///< number of nodes in tree
   /** flag to denote whether the octree extent changed (for lazy min/max eval) */
   protected boolean size_changed;

   protected Point3d tree_center = new Point3d(); // coordinate offset of tree

   protected double max_value[] = new double[3]; ///< max in x, y, z
   protected double min_value[] = new double[3]; ///< min in x, y, z

   /// data structure for ray casting, array for multithreading
   protected List<KeyRay> keyrays = new ArrayList<>();

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
      this.treeDepth = tree_depth;
      this.treeMaximumValue = tree_max_val;
      treeSize = 0;

      init();
      // no longer create an empty root node - only on demand
   }

   public OcTreeBaseImpl(OcTreeBaseImpl<NODE> other)
   {
      resolution = other.resolution;
      treeDepth = other.treeDepth;
      treeMaximumValue = other.treeMaximumValue;
      init();
      if (other.root != null)
         root = other.root.cloneRecursive();
   }

   /**
    * Swap contents of two octrees, i.e., only the underlying
    * pointer / tree structure. You have to ensure yourself that the
    * metadata (resolution etc) matches. No memory is cleared
    * in this function
    */
   public void swapContent(OcTreeBaseImpl<NODE> other)
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
   public boolean epsilonEquals(OcTreeBaseImpl<NODE> other)
   {
      if (treeDepth != other.treeDepth || treeMaximumValue != other.treeMaximumValue || resolution != other.resolution || treeSize != other.treeSize)
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

   public String getTreeType()
   {
      return "OcTreeBaseImpl";
   }

   /// Change the resolution of the octree, scaling all voxels.
   /// This will not preserve the (metric) scale!
   public void setResolution(double r)
   {
      resolution = r;

      tree_center.x = tree_center.y = tree_center.z = (float) (treeMaximumValue * resolution);

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

   public int getTreeMaximumValue()
   {
      return treeMaximumValue;
   }

   public double getNodeSize(int depth)
   {
      return OcTreeCoordinateConversionTools.computeNodeSize(depth, resolution, treeDepth);
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
   public NODE createNodeChild(NODE node, int childIndex)
   {
      OcTreeNodeTools.checkChildIndex(childIndex);
      if (!node.hasArrayForChildren())
         node.allocateChildren();

      if (node.getChildUnsafe(childIndex) != null)
         throw new RuntimeException("Something went wrong.");
      NODE newChildNode = node.create();
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

      node.removeChild(childIndex);

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
      // all children must exist, must not have children of
      // their own and have the same occupancy probability
      if (!OcTreeNodeTools.nodeChildExists(node, 0))
         return false;

      NODE firstChild = OcTreeNodeTools.getNodeChild(node, 0);
      if (firstChild.hasAtLeastOneChild())
         return false;

      for (int i = 1; i < 8; i++)
      {
         if (!OcTreeNodeTools.nodeChildExists(node, i))
            return false;

         NODE currentChild = OcTreeNodeTools.getNodeChild(node, i);

         if (currentChild.hasAtLeastOneChild() || !currentChild.epsilonEquals(firstChild))
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
   public NODE search(OcTreeKey key)
   {
      return OcTreeSearchTools.search(root, key, treeDepth);
   }

   public NODE search(OcTreeKey key, int depth)
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
      OcTreeKey key = convertCartesianCoordinateToKey(x, y, z);
      if (key == null)
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

   public boolean deleteNode(Point3d coord, int depth)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(coord);
      if (key == null)
      {
         PrintTools.error(this, "Error in deleteNode: [" + coord + "] is out of OcTree bounds!");
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
         depth = treeDepth;

      return deleteNodeRecurs(root, 0, depth, key);
   }

   /// Deletes the complete tree structure
   public void clear()
   {
      if (root != null)
      {
         deleteNodeRecurs(root);
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
      MathTools.checkIfLessOrEqual(depth, treeDepth);
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

      ray.clear();

      OcTreeKey key_origin = convertCartesianCoordinateToKey(origin);
      OcTreeKey key_end = convertCartesianCoordinateToKey(end);
      if (key_origin == null || key_end == null)
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
            tMax[i] = Double.POSITIVE_INFINITY;
            tDelta[i] = Double.POSITIVE_INFINITY;
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

         if (current_key.k[dim] >= 2 * treeMaximumValue)
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

         assert ray.size() < ray.sizeMax() - 1;

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

   @Override
   public Iterator<OcTreeSuperNode<NODE>> iterator()
   {
      return leafIterable().iterator(); // TODO Organize imports;
   }

   public Iterable<OcTreeSuperNode<NODE>> leafIterable()
   {
      return new LeafIterable<>(this); // TODO Organize imports;
   }

   public Iterator<OcTreeSuperNode<NODE>> treeIterator()
   {
      return treeIterable().iterator();
   }

   public Iterable<OcTreeSuperNode<NODE>> treeIterable()
   {
      return new OcTreeIterable<>(this); // TODO Organize imports;
   }

   public Iterable<OcTreeSuperNode<NODE>> leafBoundingBoxIterable(OcTreeKey min, OcTreeKey max)
   {
      return new LeafBoundingBoxIterable<>(this, min, max, 0); // TODO Organize imports;
   }

   //
   // Key / coordinate conversion functions
   //

   /// Converts from a single coordinate into a discrete key
   public int convertCartesianCoordinateToKey(double coordinate)
   {
      return OcTreeCoordinateConversionTools.convertCartesianCoordinateToKey(coordinate, resolution, treeDepth);
   }

   /// Converts from a single coordinate into a discrete key at a given depth
   public int convertCartesianCoordinateToKey(double coordinate, int depth)
   {
      return OcTreeCoordinateConversionTools.convertCartesianCoordinateToKey(coordinate, depth, coordinate, treeDepth);
   }

   /// Converts from a 3D coordinate into a 3D addressing key
   public OcTreeKey convertCartesianCoordinateToKey(Point3d coord)
   {
      return OcTreeCoordinateConversionTools.convertCartesianCoordinateToKey(coord, resolution, treeDepth);
   }

   /// Converts from a 3D coordinate into a 3D addressing key
   public OcTreeKey convertCartesianCoordinateToKey(double x, double y, double z)
   {
      return OcTreeCoordinateConversionTools.convertCartesianCoordinateToKey(x, y, z, resolution, treeDepth);
   }

   /// Converts from a 3D coordinate into a 3D addressing key at a given depth
   public OcTreeKey convertCartesianCoordinateToKey(Point3d coord, int depth)
   {
      return OcTreeCoordinateConversionTools.convertCartesianCoordinateToKey(coord, depth, resolution, treeDepth);
   }

   /// Converts from a 3D coordinate into a 3D addressing key at a given depth
   public OcTreeKey convertCartesianCoordinateToKey(double x, double y, double z, int depth)
   {
      return OcTreeCoordinateConversionTools.convertCartesianCoordinateToKey(x, y, z, depth, resolution, treeDepth);
   }

   public boolean convertCartesianCoordinateToKey(Point3d coord, OcTreeKey keyToPack)
   {
      return OcTreeCoordinateConversionTools.convertCartesianCoordinateToKey(coord, resolution, treeDepth, keyToPack);
   }

   /// converts from a discrete key at a given depth into a coordinate
   /// corresponding to the key's center
   public double keyToCoord(int key, int depth)
   {
      return OcTreeCoordinateConversionTools.convertKeyToCartesianCoordinate(key, depth, resolution, treeDepth);
   }

   /// converts from a discrete key at the lowest tree level into a coordinate
   /// corresponding to the key's center
   public double keyToCoord(int key)
   {
      return OcTreeCoordinateConversionTools.convertKeyToCartesianCoordinate(key, resolution, treeDepth);
   }

   /// converts from an addressing key at the lowest tree level into a coordinate
   /// corresponding to the key's center
   public Point3d keyToCoord(OcTreeKey key)
   {
      return OcTreeCoordinateConversionTools.convertKeyToCartesianCoordinate(key, resolution, treeDepth);
   }

   /// converts from an addressing key at a given depth into a coordinate
   /// corresponding to the key's center
   public Point3d keyToCoord(OcTreeKey key, int depth)
   {
      return OcTreeCoordinateConversionTools.convertKeyToCartesianCoordinate(key, depth, resolution, treeDepth);
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
            NODE child = OcTreeNodeTools.getNodeChild(node, i);
            if (child != null)
               deleteNodeRecurs(child);
         }
         node.removeChildren();
      } // else: node has no children
   }

   /// recursive call of deleteNode()
   protected boolean deleteNodeRecurs(NODE node, int depth, int max_depth, OcTreeKey key)
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

   /// recursive call of expand()
   protected void expandRecurs(NODE node, int depth, int max_depth)
   {
      if (depth >= max_depth)
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
            expandRecurs(OcTreeNodeTools.getNodeChild(node, i), depth + 1, max_depth);
         }
      }
   }

   protected int getNumLeafNodesRecurs(NODE parent)
   {
      if (parent == null)
         throw new RuntimeException("The given parent node is null");

      if (!parent.hasAtLeastOneChild()) // this is a leaf -> terminate
         return 1;

      int sum_leafs_children = 0;
      for (int i = 0; i < 8; ++i)
      {
         if (OcTreeNodeTools.nodeChildExists(parent, i))
         {
            sum_leafs_children += getNumLeafNodesRecurs(OcTreeNodeTools.getNodeChild(parent, i));
         }
      }
      return sum_leafs_children;
   }

   protected abstract NODE createRootNode();
}
