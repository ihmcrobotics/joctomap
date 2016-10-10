package us.ihmc.octoMap.ocTree;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.ColorOcTreeNode;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOccupancyOcTree;
import us.ihmc.octoMap.tools.OcTreeNodeTools;

public class ColorOcTree extends AbstractOccupancyOcTree<ColorOcTreeNode>
{
   public ColorOcTree(double resolution)
   {
      super(resolution);
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
   public boolean pruneNode(ColorOcTreeNode node, double epsilon)
   {
      if (!OcTreeNodeTools.isNodeCollapsible(node, epsilon))
         return false;

      // set value to children's values (all assumed equal)
      node.copyData(node.getChild(0));

      if (node.isColorSet())
         node.setColor(node.getAverageChildColor());

      // delete children
      for (int i = 0; i < 8; i++)
      {
         deleteNodeChild(node, i);
      }
      node.removeChildren();

      return true;
   }

   // set node color at given key or coordinate. Replaces previous color.
   public ColorOcTreeNode setNodeColor(OcTreeKeyReadOnly key, int red, int green, int blue)
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
      OcTreeKey key = coordinateToKey(x, y, z);
      if (key == null)
         return null;
      return setNodeColor(key, red, green, blue);
   }

   // integrate color measurement at given key or coordinate. Average with previous color
   public ColorOcTreeNode averageNodeColor(OcTreeKeyReadOnly key, int red, int green, int blue)
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
      OcTreeKey key = coordinateToKey(x, y, z);
      if (key == null)
         return null;
      return averageNodeColor(key, red, green, blue);
   }

   // integrate color measurement at given key or coordinate. Average with previous color
   public ColorOcTreeNode integrateNodeColor(OcTreeKeyReadOnly key, int red, int green, int blue)
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
      OcTreeKey key = coordinateToKey(x, y, z);
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
               ColorOcTreeNode childNode = node.getChild(i);
               if (childNode != null)
                  updateInnerOccupancyRecurs(childNode, depth + 1);
            }
         }
         node.updateOccupancyChildren();
         node.updateColorChildren();
      }
   }

   @Override
   protected Class<ColorOcTreeNode> getNodeClass()
   {
      return ColorOcTreeNode.class;
   }
}
