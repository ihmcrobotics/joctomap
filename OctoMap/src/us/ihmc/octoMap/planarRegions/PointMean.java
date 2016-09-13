package us.ihmc.octoMap.planarRegions;

import javax.vecmath.Point3d;

import org.apache.commons.math3.stat.descriptive.moment.Mean;

public class PointMean extends Point3d
{
   private static final long serialVersionUID = -3110940850302600107L;

   private final Mean meanX = new Mean();
   private final Mean meanY = new Mean();
   private final Mean meanZ = new Mean();

   public PointMean()
   {
   }

   public void update(Point3d point)
   {
      meanX.increment(point.getX());
      meanY.increment(point.getY());
      meanZ.increment(point.getZ());
      setX(meanX.getResult());
      setY(meanY.getResult());
      setZ(meanZ.getResult());
   }
}
