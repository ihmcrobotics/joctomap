package us.ihmc.octoMap.ocTree;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.node.ColorOcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTreeBase;

public class ColorOcTree extends AbstractOccupancyOcTreeBase<ColorOcTreeNode>
{
   public ColorOcTree(double resolution)
   {
      super(resolution);
   }

   @Override
   protected ColorOcTreeNode createRootNode()
   {
      return new ColorOcTreeNode();
   }

   /// virtual constructor: creates a new object of same type
   /// (Covariant return type requires an up-to-date compiler)
   public ColorOcTree create()
   {
      return new ColorOcTree(resolution);
   }

   /**
   * Prunes a node when it is collapsible. This overloaded
   * version only considers the node occupancy for pruning,
   * different colors of child nodes are ignored.
   * @return true if pruning was successful
   */
   @Override
   public boolean pruneNode(ColorOcTreeNode node)
   {
      if (!isNodeCollapsible(node))
         return false;

      // set value to children's values (all assumed equal)
      node.copyData(OcTreeNodeTools.getNodeChild(node, 0));

      if (node.isColorSet()) // TODO check
         node.setColor(node.getAverageChildColor());

      // delete children
      for (int i = 0; i < 8; i++)
      {
         deleteNodeChild(node, i);
      }
      node.removeChildren();

      return true;
   }

   @Override
   public boolean isNodeCollapsible(ColorOcTreeNode node)
   {
      // all children must exist, must not have children of
      // their own and have the same occupancy probability
      if (!OcTreeNodeTools.nodeChildExists(node, 0))
         return false;

      ColorOcTreeNode firstChild = OcTreeNodeTools.getNodeChild(node, 0);
      if (firstChild.hasAtLeastOneChild())
         return false;

      for (int i = 1; i < 8; i++)
      {
         // compare nodes only using their occupancy, ignoring color for pruning
         if (!OcTreeNodeTools.nodeChildExists(node, i))
            return false;
         ColorOcTreeNode child = OcTreeNodeTools.getNodeChild(node, i);
         if (child.hasAtLeastOneChild() || !child.epsilonEquals(firstChild))
            return false;
      }

      return true;
   }

   // set node color at given key or coordinate. Replaces previous color.
   public ColorOcTreeNode setNodeColor(OcTreeKey key, int red, int green, int blue)
   {
      ColorOcTreeNode n = search(key);
      if (n != null)
      {
         n.setColor(red, green, blue);
      }
      return n;
   }

   public ColorOcTreeNode setNodeColor(double x, double y, double z, int red, int green, int blue)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(x, y, z);
      if (key == null)
         return null;
      return setNodeColor(key, red, green, blue);
   }

   // integrate color measurement at given key or coordinate. Average with previous color
   public ColorOcTreeNode averageNodeColor(OcTreeKey key, int red, int green, int blue)
   {
      ColorOcTreeNode node = search(key);
      if (node != null)
      {
         if (node.isColorSet())
         {
            node.interpolateColor(0.5, red, green, blue);
         }
         else
         {
            node.setColor(red, green, blue);
         }
      }
      return node;
   }

   public ColorOcTreeNode averageNodeColor(double x, double y, double z, int red, int green, int blue)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(x, y, z);
      if (key == null)
         return null;
      return averageNodeColor(key, red, green, blue);
   }

   // integrate color measurement at given key or coordinate. Average with previous color
   public ColorOcTreeNode integrateNodeColor(OcTreeKey key, int red, int green, int blue)
   {
      ColorOcTreeNode node = search(key);
      if (node != null)
      {
         if (node.isColorSet())
         {
            node.integrateColor(red, green, blue);
         }
         else
         {
            node.setColor(red, green, blue);
         }
      }
      return node;
   }

   public ColorOcTreeNode integrateNodeColor(double x, double y, double z, int red, int green, int blue)
   {
      OcTreeKey key = convertCartesianCoordinateToKey(x, y, z);
      if (key == null)
         return null;
      return integrateNodeColor(key, red, green, blue);
   }

   // update inner nodes, sets color to average child color
   @Override
   public void updateInnerOccupancy()
   {
      updateInnerOccupancyRecurs(root, 0);
   }

   @Override
   protected void updateInnerOccupancyRecurs(ColorOcTreeNode node, int depth)
   {
      // only recurse and update for inner nodes:
      if (node != null && node.hasAtLeastOneChild())
      {
         // return early for last level:
         if (depth < treeDepth)
         {
            for (int i = 0; i < 8; i++)
            {
               if (OcTreeNodeTools.nodeChildExists(node, i))
               {
                  updateInnerOccupancyRecurs(OcTreeNodeTools.getNodeChild(node, i), depth + 1);
               }
            }
         }
         node.updateOccupancyChildren();
         node.updateColorChildren();
      }
   }
}
