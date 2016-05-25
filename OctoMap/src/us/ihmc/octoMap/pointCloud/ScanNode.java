package us.ihmc.octoMap.pointCloud;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * A 3D scan as Pointcloud, performed from a Pose6D.
 */
public class ScanNode
{
   private PointCloud scan;
   private final RigidBodyTransform pose = new RigidBodyTransform(); ///< 6D pose from which the scan was performed
   private int id;

   public ScanNode(PointCloud scan, RigidBodyTransform pose, int id)
   {
      this.scan = new PointCloud(scan);
      this.pose.set(pose);
      this.id = id;
   }

   public ScanNode()
   {
      scan = null;
   }

   public ScanNode(ScanNode other)
   {
      this.scan = new PointCloud(other.scan);
      this.pose.set(other.pose);
      this.id = other.id;
   }

   public PointCloud getScan()
   {
      return scan;
   }

   public int getScanSize()
   {
      return scan.size();
   }

   public void cropScan(Point3d lowerBound, Point3d upperBound)
   {
      scan.crop(lowerBound, upperBound);
   }

   public void setScan(PointCloud scan)
   {
      this.scan = scan;
   }

   public void transformAbsoluteScan(RigidBodyTransform transform)
   {
      scan.transformAbsolute(transform);
   }

   public RigidBodyTransform getPose()
   {
      return pose;
   }

   public int getId()
   {
      return id;
   }

   public boolean equals(ScanNode other)
   {
      return id == other.id;
   }
}