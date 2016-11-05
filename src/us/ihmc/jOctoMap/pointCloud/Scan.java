package us.ihmc.jOctoMap.pointCloud;

import javax.vecmath.Point3d;

public class Scan
{
   private final Point3d sensorOrigin;
   private final PointCloud pointCloud;

   public Scan(Point3d sensorOrigin, PointCloud pointCloud)
   {
      this.sensorOrigin = sensorOrigin;
      this.pointCloud = pointCloud;
   }

   public Point3d getSensorOrigin()
   {
      return sensorOrigin;
   }

   public PointCloud getPointCloud()
   {
      return pointCloud;
   }

   public int getNumberOfPoints()
   {
      return pointCloud.getNumberOfPoints();
   }
}
