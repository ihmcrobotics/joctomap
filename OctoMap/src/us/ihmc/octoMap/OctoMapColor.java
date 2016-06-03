package us.ihmc.octoMap;

public class OctoMapColor
{
   private int red, green, blue, opacity;

   public OctoMapColor()
   {
      this(255, 255, 255, 255);
   }

   public OctoMapColor(int red, int green, int blue)
   {
      this(red, green, blue, 255);
   }

   public OctoMapColor(int red, int green, int blue, int opacity)
   {
      set(red, green, blue, opacity);
   }

   public OctoMapColor(OctoMapColor other)
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

   public void set(OctoMapColor other)
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
      checkIfAlphaValid(alpha);
      this.red = interpolate(alpha, this.red, red);
      this.green = interpolate(alpha, this.green, green);
      this.blue = interpolate(alpha, this.blue, blue);
      this.opacity = interpolate(alpha, this.opacity, opacity);
   }

   public void interpolate(double alpha, OctoMapColor color)
   {
      interpolate(alpha, this, color);
   }

   public void interpolate(double alpha, OctoMapColor firstColor, OctoMapColor secondColor)
   {
      checkIfAlphaValid(alpha);
      red = interpolate(alpha, firstColor.red, secondColor.red);
      green = interpolate(alpha, firstColor.green, secondColor.green);
      blue = interpolate(alpha, firstColor.blue, secondColor.blue);
      opacity = interpolate(alpha, firstColor.opacity, secondColor.opacity);
   }

   private static void checkIfAlphaValid(double alpha)
   {
      if (alpha < 0.0 || alpha > 1.0)
         throw new RuntimeException("Invalid interpolation alpha: " + alpha);
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

   public boolean equals(OctoMapColor other)
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