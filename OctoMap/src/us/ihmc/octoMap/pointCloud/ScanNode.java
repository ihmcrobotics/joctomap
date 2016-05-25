package us.ihmc.octoMap.pointCloud;

import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * A 3D scan as Pointcloud, performed from a Pose6D.
 */
public class ScanNode
{
   PointCloud scan;
   RigidBodyTransform pose = new RigidBodyTransform(); ///< 6D pose from which the scan was performed
   int id;

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