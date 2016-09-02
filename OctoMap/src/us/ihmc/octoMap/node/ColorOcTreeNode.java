package us.ihmc.octoMap.node;

import us.ihmc.octoMap.OctoMapColor;

public final class ColorOcTreeNode extends AbstractOccupancyOcTreeNode<ColorOcTreeNode>
{
   private final OctoMapColor color = new OctoMapColor(255, 255, 255);

   public ColorOcTreeNode()
   {
      super();
   }

   @Override
   public void clear()
   {
      super.resetLogOdds();
      color.set(255, 255, 255);
   }

   public void setColor(OctoMapColor color)
   {
      this.color.set(color);
   }

   public void setColor(int red, int green, int blue)
   {
      color.set(red, green, blue);
   }

   public OctoMapColor getColor()
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
      return (int) (currentColorParameter * occupancy + newColorParameter * (0.99 - occupancy));
   }

   @Override
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

   public OctoMapColor getAverageChildColor()
   {
      int meanRed = 0;
      int meanGreen = 0;
      int meanBlue = 0;
      int count = 0;

      if (children != null)
      {
         for (int i = 0; i < 8; i++)
         {
            ColorOcTreeNode child = children[i];

            if (child != null && child.isColorSet())
            {
               OctoMapColor childColor = child.getColor();
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
         return new OctoMapColor(meanRed, meanGreen, meanBlue);
      }
      else
      { // no child had a color other than white
         return new OctoMapColor(255, 255, 255);
      }
   }
}