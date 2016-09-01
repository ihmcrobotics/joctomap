package us.ihmc.octoMap.ocTree;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class PlanarRegion
{
   public static final int NO_REGION_ID = Integer.MIN_VALUE;

   private int id = NO_REGION_ID;

   private final VectorMean normal = new VectorMean();
   private final PointMean point = new PointMean();
   private final Vector3d temporaryVector = new Vector3d();

   public PlanarRegion(int id)
   {
      this.id = id;
   }

   public void update(Vector3d normal, Point3d point)
   {
      this.normal.update(normal);
      this.point.update(point);
   }

   public double distance(Point3d point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.point);
      return temporaryVector.dot(this.normal);
   }

   public double absoluteDistance(Point3d point)
   {
      return Math.abs(distance(point));
   }

   public double angle(Vector3d normal)
   {
      return this.normal.angle(normal);
   }

   public double absoluteAngle(Vector3d normal)
   {
      return Math.abs(angle(normal));
   }

   public int getId()
   {
      return id;
   }

   public int getNumberOfNodes()
   {
      return normal.getNumberOfSamples();
   }
}
