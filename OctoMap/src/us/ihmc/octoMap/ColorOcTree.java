package us.ihmc.octoMap;

import us.ihmc.octoMap.ColorOcTree.ColorOcTreeNode;
import us.ihmc.octoMap.node.OcTreeDataNode;
import us.ihmc.octoMap.node.OcTreeNode;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.robotics.MathTools;

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
         OcTreeDataNode<Float> child = OcTreeNodeTools.getNodeChild(node, i);
         if (child.hasAtLeastOneChild() || !(child.epsilonEquals(firstChild)))
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
   public void updateInnerOccupancy()
   {
      updateInnerOccupancyRecurs(root, 0);
   }

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

   public static class ColorOcTreeNode extends OcTreeNode
   {
      private final Color color = new Color(255, 255, 255);

      public ColorOcTreeNode()
      {
         super();
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

      public void interpolateColor(double alpha, int red, int green, int blue)
      {
         color.interpolate(alpha, red, green, blue);
      }

      public void integrateColor(int red, int green, int blue)
      {
         double occupancyProbability = getOccupancy();
         color.setRed(integrateColorParameter(color.getRed(), red, occupancyProbability));
         color.setGreen(integrateColorParameter(color.getGreen(), green, occupancyProbability));
         color.setBlue(integrateColorParameter(color.getBlue(), blue, occupancyProbability));
      }

      private static int integrateColorParameter(int currentColorParameter, int newColorParameter, double occupancy)
      {
         return (int) ((double) currentColorParameter * occupancy + (double) newColorParameter * (0.99 - occupancy));
      }

      public void copyData(ColorOcTreeNode other)
      {
         super.copyData(other);
         setColor(other.color);
      }

      // has any color been integrated? (pure white is very unlikely...)
      public boolean isColorSet()
      {
         return color.getRed() != 255 || color.getGreen() != 255 || color.getBlue() != 255;
      }

      public void updateColorChildren()
      {
         color.set(getAverageChildColor());
      }

      public Color getAverageChildColor()
      {
         int meanRed = 0;
         int meanGreen = 0;
         int meanBlue = 0;
         int count = 0;

         if (children != null)
         {
            for (int i = 0; i < 8; i++)
            {
               ColorOcTreeNode child = (ColorOcTreeNode) children[i];

               if (child != null && child.isColorSet())
               {
                  Color childColor = child.getColor();
                  meanRed += childColor.getRed();
                  meanGreen += childColor.getGreen();
                  meanBlue += childColor.getBlue();
                  ++count;
               }
            }
         }

         if (count > 0)
         {
            meanRed /= count;
            meanGreen /= count;
            meanBlue /= count;
            return new Color(meanRed, meanGreen, meanBlue);
         }
         else
         { // no child had a color other than white
            return new Color(255, 255, 255);
         }
      }
   }

   public static class Color
   {
      private int red, green, blue, opacity;

      public Color()
      {
         this(255, 255, 255, 255);
      }

      public Color(int red, int green, int blue)
      {
         this(red, green, blue, 255);
      }

      public Color(int red, int green, int blue, int opacity)
      {
         set(red, green, blue, opacity);
      }

      public Color(Color other)
      {
         set(other);
      }

      public void set(int red, int green, int blue)
      {
         set(red, green, blue, 255);
      }

      public void set(int red, int green, int blue, int opacity)
      {
         checkColor(red, green, blue, opacity);
         this.red = red;
         this.green = green;
         this.blue = blue;
         this.opacity = opacity;
      }

      public void set(Color other)
      {
         red = other.red;
         green = other.green;
         blue = other.blue;
         opacity = other.opacity;
      }

      public void setRed(int red)
      {
         checkRedValue(red);
         this.red = red;
      }

      public void setGreen(int green)
      {
         checkGreenValue(green);
         this.green = green;
      }

      public void setBlue(int blue)
      {
         checkBlueValue(blue);
         this.blue = blue;
      }

      public void setOpacity(int opacity)
      {
         checkOpacityValue(opacity);
         this.opacity = opacity;
      }

      public void interpolate(double alpha, int red, int green, int blue)
      {
         interpolate(alpha, red, green, blue, 255);
      }

      public void interpolate(double alpha, int red, int green, int blue, int opacity)
      {
         MathTools.checkIfInRange(alpha, 0.0, 1.0);
         this.red = interpolate(alpha, this.red, red);
         this.green = interpolate(alpha, this.green, green);
         this.blue = interpolate(alpha, this.blue, blue);
         this.opacity = interpolate(alpha, this.opacity, opacity);
      }

      public void interpolate(double alpha, Color color)
      {
         interpolate(alpha, this, color);
      }

      public void interpolate(double alpha, Color firstColor, Color secondColor)
      {
         MathTools.checkIfInRange(alpha, 0.0, 1.0);
         red = interpolate(alpha, firstColor.red, secondColor.red);
         green = interpolate(alpha, firstColor.green, secondColor.green);
         blue = interpolate(alpha, firstColor.blue, secondColor.blue);
         opacity = interpolate(alpha, firstColor.opacity, secondColor.opacity);
      }

      private static int interpolate(double alpha, int firstInteger, int secondInteger)
      {
         return (int) ((1.0 - alpha) * firstInteger + alpha * secondInteger);
      }

      public int getRed()
      {
         return red;
      }

      public int getGreen()
      {
         return green;
      }

      public int getBlue()
      {
         return blue;
      }

      public int getOpacity()
      {
         return opacity;
      }

      public boolean equals(Color other)
      {
         return red == other.red && green == other.green && blue == other.blue && opacity == other.opacity;
      }

      private static final String RED_STR = "red";
      private static final String GREEN_STR = "green";
      private static final String BLUE_STR = "blue";
      private static final String OPACITY_STR = "opacity";

      private static final void checkColor(int red, int green, int blue, int opacity)
      {
         checkRedValue(red);
         checkGreenValue(green);
         checkBlueValue(blue);
         checkOpacityValue(opacity);
      }

      private static final void checkRedValue(int red)
      {
         checkParameterRange(RED_STR, red);
      }

      private static final void checkGreenValue(int green)
      {
         checkParameterRange(GREEN_STR, green);
      }

      private static final void checkBlueValue(int blue)
      {
         checkParameterRange(BLUE_STR, blue);
      }

      private static final void checkOpacityValue(int opacity)
      {
         checkParameterRange(OPACITY_STR, opacity);
      }

      private static final void checkParameterRange(String parameterName, int value)
      {
         if (value < 0 || value > 255)
            throw new IllegalArgumentException("Color parameter " + parameterName + " outside of expected range:" + value);
      }
   }
}
