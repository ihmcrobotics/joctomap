package us.ihmc.octoMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;

public class Pointcloud implements Iterable<Point3d>
{
   protected RigidBodyTransform current_inv_transform = new RigidBodyTransform();
   protected List<Point3d> points = new ArrayList<>();

   /**
    * A collection of 3D coordinates (point3d), which are regarded as endpoints of a
    * 3D laser scan.
    */
   public Pointcloud()
   {
   }

   public Pointcloud(Pointcloud other)
   {
      push_back(other);
   }

   public int size()
   {
      return points.size();
   }

   public void clear()
   {
      // delete the points
      if (!points.isEmpty())
      {
         points.clear();
      }
   }

   public void push_back(double x, double y, double z)
   {
      points.add(new Point3d(x, y, z));
   }

   public void push_back(Point3d p)
   {
      points.add(p);
   }

   /// Add points from other Pointcloud
   public void push_back(Pointcloud other)
   {
      for (Point3d point : other)
         push_back(new Point3d(point));
   }

   /// Apply transform to each point
   public void transform(RigidBodyTransform transform)
   {
      for (int i = 0; i < size(); i++)
         transform.transform(points.get(i));

      // FIXME: not correct for multiple transforms
       current_inv_transform.invert(transform);
   }

   /// Rotate each point in pointcloud
   public void rotate(double roll, double pitch, double yaw)
   {
      Matrix3d rotation = new Matrix3d();
      RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, rotation);

      for (int i = 0; i < size(); i++)
         rotation.transform(points.get(i));

   }

   /// Apply transform to each point, undo previous transforms
   public void transformAbsolute(RigidBodyTransform transform)
   {

      // undo previous transform, then apply current transform
      current_inv_transform.multiply(transform);

      for (int i = 0; i < size(); i++)
         current_inv_transform.transform(points.get(i));

      current_inv_transform.invert(transform);
   }

   /// Calculate bounding box of Pointcloud
   public void calcBBX(Point3d lowerBound, Point3d upperBound)
   {

      double min_x, min_y, min_z;
      double max_x, max_y, max_z;
      min_x = min_y = min_z = 1e6;
      max_x = max_y = max_z = -1e6;

      double x,y,z;

      for (Point3d point : this) {

        x = point.x;
        y = point.y;
        z = point.z;

        if (x < min_x) min_x = x;
        if (y < min_y) min_y = y;
        if (z < min_z) min_z = z;

        if (x > max_x) max_x = x;
        if (y > max_y) max_y = y;
        if (z > max_z) max_z = z;
      }

      lowerBound.x = min_x; lowerBound.y = min_y; lowerBound.z = min_z;
      upperBound.x = max_x; upperBound.y = max_y; upperBound.z = max_z;
   }

   /// Crop Pointcloud to given bounding box
   public void crop(Point3d lowerBound, Point3d upperBound)
   {
      Pointcloud result = new Pointcloud();

      double min_x, min_y, min_z;
      double max_x, max_y, max_z;
      double x,y,z;

      min_x = lowerBound.x; min_y = lowerBound.y; min_z = lowerBound.z;
      max_x = upperBound.x; max_y = upperBound.y; max_z = upperBound.z;

      for (Point3d point : this) {

        x = point.x;
        y = point.y;
        z = point.z;

        if ( (x >= min_x) &&
        (y >= min_y) &&
        (z >= min_z) &&
        (x <= max_x) &&
        (y <= max_y) &&
        (z <= max_z) ) {
     result.push_back (x,y,z);
        }
      } // end for points

      clear();
      push_back(result);
   }

   // removes any points closer than [thres] to (0,0,0)
   public void minDist(double thres)
   {
      Pointcloud result = new Pointcloud();

      double x,y,z;
      for (Point3d point : this) {
         x = point.x;
         y = point.y;
         z = point.z;
        double dist = Math.sqrt(x*x+y*y+z*z);
        if ( dist > thres ) result.push_back (x,y,z);
      } // end for points
      clear();
      push_back(result);
    }

   public void subSampleRandom(int num_samples, Pointcloud sample_cloud)
   {
      sample_cloud = new Pointcloud(this);
      Random random = new Random();
      
      while (sample_cloud.size() > num_samples)
      {
         int indexToRemove = random.nextInt(sample_cloud.size());
         int lastIndex = sample_cloud.size() - 1;
         Collections.swap(sample_cloud.points, indexToRemove, lastIndex);
         sample_cloud.points.remove(lastIndex);
      }
    }

   public Point3d back()
   {
      return points.get(points.size() - 1);
   }

   /// Returns a copy of the ith point in point cloud.
   /// Use operator[] for direct access to point reference.
   public Point3d getPoint(int i) // may return NULL
   {
      return points.get(i);
   }

   @Override
   public Iterator<Point3d> iterator()
   {
      return points.iterator();
   }
}
