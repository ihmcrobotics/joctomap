package us.ihmc.octoMap;

public class ColorOcTree extends occup
{

   public ColorOcTree()
   {
      
   }

   public abstract static class ColorOcTreeNode extends OcTreeNode
   {
      private Color color = new Color();

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

      public abstract void updateColorChildren();

      public abstract Color getAverageChildColor();
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
