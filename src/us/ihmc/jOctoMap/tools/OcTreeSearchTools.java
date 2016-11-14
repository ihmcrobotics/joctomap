package us.ihmc.jOctoMap.tools;

import static us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools.*;

import javax.vecmath.Point3d;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;

public abstract class OcTreeSearchTools
{
   /** 
    *  Search node at specified depth given a 3d point (depth=0: search full tree depth).
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, double x, double y, double z, double resolution, int treeDepth)
   {
      return search(rootNode, x, y, z, 0, resolution, treeDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, double x, double y, double z, int depth, double resolution, int treeDepth)
   {
      OcTreeKey key = coordinateToKey(x, y, z, resolution, treeDepth);
      if (key == null)
      {
         System.err.println(OcTreeSearchTools.class.getSimpleName() + ": Error in search: [" + x + " " + y + " " + z + "] is out of OcTree bounds!");
         return null;
      }
      else
      {
         return search(rootNode, key, depth, treeDepth);
      }
   }

   /**
    *  Search node at specified depth given a 3d point (depth=0: search full tree depth)
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, Point3d coord, double resolution, int treeDepth)
   {
      return search(rootNode, coord, 0, resolution, treeDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, Point3d coord, int depth, double resolution, int treeDepth)
   {
      OcTreeKey key = coordinateToKey(coord, resolution, treeDepth);
      if (key == null)
      {
         System.err.println(OcTreeSearchTools.class.getSimpleName() + ": Error in search: [" + coord + "] is out of OcTree bounds!");
         return null;
      }
      else
      {
         return search(rootNode, key, depth, treeDepth);
      }
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, OcTreeKeyReadOnly key, int treeDepth)
   {
      return search(rootNode, key, 0, treeDepth);
   }

   /**
    *  Search a node at specified depth given an addressing key (depth=0: search full tree depth)
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, OcTreeKeyReadOnly key, int depth, int treeDepth)
   {
      JOctoMapTools.checkIfDepthValid(depth, treeDepth);
      if (rootNode == null)
         return null;

      if (depth == 0)
         depth = treeDepth;

      int k0 = key.getKey(0);
      int k1 = key.getKey(1);
      int k2 = key.getKey(2);

      // generate appropriate keyAtDepth for queried depth
      if (depth != treeDepth)
      {
         k0 = OcTreeKeyTools.adjustKeyAtDepth(k0, depth, treeDepth);
         k1 = OcTreeKeyTools.adjustKeyAtDepth(k1, depth, treeDepth);
         k2 = OcTreeKeyTools.adjustKeyAtDepth(k2, depth, treeDepth);
      }

      NODE currentNode = rootNode;

      // follow nodes down to requested depth
      for (int currentDepth = 0; currentDepth < depth; currentDepth++)
      {
         int childIndex = OcTreeKeyTools.computeChildIndex(k0, k1, k2, currentDepth, treeDepth);
         NODE childNode;

         if ((childNode = currentNode.getChild(childIndex)) != null)
         {
            currentNode = childNode;
         }
         else
         {
            // we expected a child but did not get it
            // is the current node a leaf already?
            if (!currentNode.hasAtLeastOneChild())
            {
               return currentNode;
            }
            else
            {
               // it is not, search failed
               return null;
            }
         }
      } // end for
      return currentNode;
   }
}
