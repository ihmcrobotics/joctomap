package us.ihmc.octoMap.planarRegions;

import javax.vecmath.Vector3d;

import org.apache.commons.math3.stat.descriptive.moment.Mean;

public class VectorMean extends Vector3d
{
   private static final long serialVersionUID = 2936790417418842327L;

   private final Mean meanX = new Mean();
   private final Mean meanY = new Mean();
   private final Mean meanZ = new Mean();

   public VectorMean()
   {
   }

   public void update(Vector3d vector)
   {
      update(vector.getX(), vector.getY(), vector.getZ());
   }

   public void update(double x, double y, double z)
   {
      meanX.increment(x);
      meanY.increment(y);
      meanZ.increment(z);
      setX(meanX.getResult());
      setY(meanY.getResult());
      setZ(meanZ.getResult());
   }

   public void clear()
   {
      meanX.clear();
      meanY.clear();
      meanZ.clear();
      set(0.0, 0.0, 0.0);
   }

   public int getNumberOfSamples()
   {
      return (int) meanX.getN();
   }
}
