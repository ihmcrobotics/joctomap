package us.ihmc.jOctoMap.pointCloud;

import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;

public class Scan
{
   private final Point3D sensorOrigin;
   private final PointCloud pointCloud;

   public Scan(Point3DReadOnly sensorOrigin, PointCloud pointCloud)
   {
      this.sensorOrigin = new Point3D(sensorOrigin);
      this.pointCloud = pointCloud;
   }

   public Scan(Scan other)
   {
      sensorOrigin = new Point3D(other.sensorOrigin);
      pointCloud = new PointCloud(other.pointCloud);
   }

   public void transform(Transform transform)
   {
      transform.transform(sensorOrigin);
      pointCloud.transform(transform);
   }

   public Point3DReadOnly getSensorOrigin()
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
