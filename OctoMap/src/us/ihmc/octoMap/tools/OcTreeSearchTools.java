package us.ihmc.octoMap.tools;

import static us.ihmc.octoMap.tools.OcTreeKeyConversionTools.coordinateToKey;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;

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
      OctoMapTools.checkIfDepthValid(depth, treeDepth);
      if (rootNode == null)
         return null;

      if (depth == 0)
         depth = treeDepth;

      // generate appropriate keyAtDepth for queried depth
      OcTreeKeyReadOnly keyAtDepth;
      if (depth != treeDepth)
         keyAtDepth = OcTreeKeyTools.adjustKeyAtDepth(key, depth, treeDepth);
      else
         keyAtDepth = key;

      NODE currentNode = rootNode;

      int level = treeDepth - depth;

      // follow nodes down to requested level (for level = 0 it's the last level)
      for (int currentDepth = (treeDepth - 1); currentDepth >= level; --currentDepth)
      {
         int childIndex = OcTreeKeyTools.computeChildIndex(keyAtDepth, currentDepth);

         if (OcTreeNodeTools.nodeChildExists(currentNode, childIndex))
         {
            currentNode = OcTreeNodeTools.getNodeChild(currentNode, childIndex);
         }
         else
         {
            // we expected a child but did not get it
            // is the current node a leaf already?
            if (!currentNode.hasAtLeastOneChild())
            { // TODO similar check to nodeChildExists?
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
