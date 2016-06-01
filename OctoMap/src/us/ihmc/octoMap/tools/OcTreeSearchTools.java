package us.ihmc.octoMap.tools;

import static us.ihmc.octoMap.tools.OcTreeCoordinateConversionTools.convertCartesianCoordinateToKey;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.tools.io.printing.PrintTools;

public abstract class OcTreeSearchTools
{
   /** 
    *  Search node at specified depth given a 3d point (depth=0: search full tree depth).
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, double x, double y, double z, double resolution, int maxDepth)
   {
      return search(rootNode, x, y, z, 0, resolution, maxDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, double x, double y, double z, int depth, double resolution, int maxDepth)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(x, y, z, resolution, maxDepth);
      if (key == null)
      {
         PrintTools.error(OcTreeSearchTools.class, "Error in search: [" + x + " " + y + " " + z + "] is out of OcTree bounds!");
         return null;
      }
      else
      {
         return search(rootNode, key, depth, maxDepth);
      }
   }

   /**
    *  Search node at specified depth given a 3d point (depth=0: search full tree depth)
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, Point3d coord, double resolution, int maxDepth)
   {
      return search(rootNode, coord, 0, resolution, maxDepth);
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, Point3d coord, int depth, double resolution, int maxDepth)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(coord, resolution, maxDepth);
      if (key == null)
      {
         PrintTools.error(OcTreeSearchTools.class, "Error in search: [" + coord + "] is out of OcTree bounds!");
         return null;
      }
      else
      {
         return search(rootNode, key, depth, maxDepth);
      }
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, OcTreeKey key, int maxDepth)
   {
      return search(rootNode, key, 0, maxDepth);
   }

   /**
    *  Search a node at specified depth given an addressing key (depth=0: search full tree depth)
    *  You need to check if the returned node is NULL, since it can be in unknown space.
    *  @return pointer to node if found, NULL otherwise
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> NODE search(NODE rootNode, OcTreeKey key, int depth, int maxDepth)
   {
      MathTools.checkIfLessOrEqual(depth, maxDepth);
      if (rootNode == null)
         return null;

      if (depth == 0)
         depth = maxDepth;

      // generate appropriate keyAtDepth for queried depth
      OcTreeKey keyAtDepth;
      if (depth != maxDepth)
         keyAtDepth = OcTreeKeyTools.adjustKeyAtDepth(key, depth, maxDepth);
      else
         keyAtDepth = new OcTreeKey(key);

      NODE currentNode = rootNode;

      int level = maxDepth - depth;

      // follow nodes down to requested level (for level = 0 it's the last level)
      for (int currentDepth = (maxDepth - 1); currentDepth >= level; --currentDepth)
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
