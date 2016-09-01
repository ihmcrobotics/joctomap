package us.ihmc.octoMap.ocTree;

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
      meanX.increment(vector.getX());
      meanY.increment(vector.getY());
      meanZ.increment(vector.getZ());
      setX(meanX.getResult());
      setY(meanY.getResult());
      setZ(meanZ.getResult());
   }

   public int getNumberOfSamples()
   {
      return (int) meanX.getN();
   }
}
