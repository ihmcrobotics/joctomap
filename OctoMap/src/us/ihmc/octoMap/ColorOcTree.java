package us.ihmc.octoMap;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.ColorOcTree.ColorOcTreeNode;

public class ColorOcTree extends OccupancyOcTreeBase<ColorOcTreeNode>
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

   public String getTreeType()
   {
      return "ColorOcTree";
   }

   /**
   * Prunes a node when it is collapsible. This overloaded
   * version only considers the node occupancy for pruning,
   * different colors of child nodes are ignored.
   * @return true if pruning was successful
   */
   public boolean pruneNode(ColorOcTreeNode node)
   {
      if (!isNodeCollapsible(node))
         return false;

      // set value to children's values (all assumed equal)
      node.copyData(getNodeChild(node, 0));

      if (node.isColorSet()) // TODO check
         node.setColor(node.getAverageChildColor());

      // delete children
      for (int i = 0; i < 8; i++)
      {
         deleteNodeChild(node, i);
      }
      node.children = null;

      return true;
   }

   public boolean isNodeCollapsible(ColorOcTreeNode node)
   {
      // all children must exist, must not have children of
      // their own and have the same occupancy probability
      if (!nodeChildExists(node, 0))
         return false;

      ColorOcTreeNode firstChild = getNodeChild(node, 0);
      if (nodeHasChildren(firstChild))
         return false;

      for (int i = 1; i < 8; i++)
      {
         // compare nodes only using their occupancy, ignoring color for pruning
         if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i).getValue() == firstChild.getValue()))
            return false;
      }

      return true;
   }

   // set node color at given key or coordinate. Replaces previous color.
   public ColorOcTreeNode setNodeColor(OcTreeKey key, int r, int g, int b)
   {
      ColorOcTreeNode n = search(key);
      if (n != null)
      {
         n.setColor(r, g, b);
      }
      return n;
   }

   public ColorOcTreeNode setNodeColor(float x, float y, float z, int r, int g, int b)
   {
      OcTreeKey key = new OcTreeKey();
      if (!coordToKeyChecked(new Point3d(x, y, z), key))
         return null;
      return setNodeColor(key, r, g, b);
   }

   // integrate color measurement at given key or coordinate. Average with previous color
   public ColorOcTreeNode averageNodeColor(OcTreeKey key, int r, int g, int b)
   {
      ColorOcTreeNode n = search(key);
      if (n != null)
      {
         if (n.isColorSet())
         {
            Color prev_color = n.getColor();
            n.setColor((prev_color.red + r) / 2, (prev_color.green + g) / 2, (prev_color.blue + b) / 2);
         }
         else
         {
            n.setColor(r, g, b);
         }
      }
      return n;
   }

   public ColorOcTreeNode averageNodeColor(float x, float y, float z, int r, int g, int b)
   {
      OcTreeKey key = new OcTreeKey();
      if (!coordToKeyChecked(new Point3d(x, y, z), key))
         return null;
      return averageNodeColor(key, r, g, b);
   }

   // integrate color measurement at given key or coordinate. Average with previous color
   public ColorOcTreeNode integrateNodeColor(OcTreeKey key, int r, int g, int b)
   {
      ColorOcTreeNode n = search(key);
      if (n != null)
      {
         if (n.isColorSet())
         {
            Color prev_color = n.getColor();
            double node_prob = n.getOccupancy();
            int new_r = (int) ((double) prev_color.red * node_prob + (double) r * (0.99 - node_prob));
            int new_g = (int) ((double) prev_color.green * node_prob + (double) g * (0.99 - node_prob));
            int new_b = (int) ((double) prev_color.blue * node_prob + (double) b * (0.99 - node_prob));
            n.setColor(new_r, new_g, new_b);
         }
         else
         {
            n.setColor(r, g, b);
         }
      }
      return n;
   }

   public ColorOcTreeNode integrateNodeColor(float x, float y, float z, int r, int g, int b)
   {
      OcTreeKey key = new OcTreeKey();
      if (!coordToKeyChecked(new Point3d(x, y, z), key))
         return null;
      return integrateNodeColor(key, r, g, b);
   }

   // update inner nodes, sets color to average child color
   public void updateInnerOccupancy()
   {
      updateInnerOccupancyRecurs(root, 0);
   }

   protected void updateInnerOccupancyRecurs(ColorOcTreeNode node, int depth)
   {
      // only recurse and update for inner nodes:
      if (nodeHasChildren(node))
      {
         // return early for last level:
         if (depth < tree_depth)
         {
            for (int i = 0; i < 8; i++)
            {
               if (nodeChildExists(node, i))
               {
                  updateInnerOccupancyRecurs(getNodeChild(node, i), depth + 1);
               }
            }
         }
         node.updateOccupancyChildren();
         node.updateColorChildren();
      }
   }

   public static class ColorOcTreeNode extends OcTreeNode
   {
      private final Color color = new Color();

      public ColorOcTreeNode()
      {
         super();
      }

      public ColorOcTreeNode(ColorOcTreeNode other)
      {
         super(other);

      }

      public void setColor(Color color)
      {
         this.color.set(color);
      }

      public void setColor(int red, int green, int blue)
      {
         color.set(red, green, blue);
      }

      public Color getColor()
      {
         return color;
      }

      public void copyData(ColorOcTreeNode other)
      {
         super.copyData(other);
         setColor(other.color);
      }

      // has any color been integrated? (pure white is very unlikely...)
      public boolean isColorSet()
      {
         return color.red != 255 || color.green != 255 || color.blue != 255;
      }

      public void updateColorChildren()
      {
         color.set(getAverageChildColor());
      }

      public Color getAverageChildColor()
      {
         int mr = 0;
         int mg = 0;
         int mb = 0;
         int c = 0;

         if (children != null)
         {
            for (int i = 0; i < 8; i++)
            {
               ColorOcTreeNode child = (ColorOcTreeNode) children[i];

               if (child != null && child.isColorSet())
               {
                  mr += child.getColor().red;
                  mg += child.getColor().green;
                  mb += child.getColor().blue;
                  ++c;
               }
            }
         }

         if (c > 0)
         {
            mr /= c;
            mg /= c;
            mb /= c;
            return new Color(mr, mg, mb);
         }
         else
         { // no child had a color other than white
            return new Color(255, 255, 255);
         }
      }
   }

   public static class Color
   {
      private int red, green, blue;

      public Color()
      {
         this(255, 255, 255);
      }

      public Color(int red, int green, int blue)
      {
         set(red, green, blue);
      }

      public Color(Color other)
      {
         set(other);
      }

      public void set(int red, int green, int blue)
      {
         this.red = red;
         this.green = green;
         this.blue = blue;
      }

      public void set(Color other)
      {
         red = other.red;
         green = other.green;
         blue = other.blue;
      }

      public boolean equals(Color other)
      {
         return red == other.red && green == other.green && blue == other.blue;
      }
   }
}
