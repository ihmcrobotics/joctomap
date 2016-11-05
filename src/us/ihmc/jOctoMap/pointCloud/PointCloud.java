package us.ihmc.jOctoMap.pointCloud;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Stream;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class PointCloud implements Iterable<Point3f>
{
   protected final List<Point3f> points = new ArrayList<>();

   /**
    * A collection of 3D coordinates (point3d), which are regarded as endpoints of a
    * 3D laser scan.
    */
   public PointCloud()
   {
   }

   public PointCloud(PointCloud other)
   {
      addAll(other);
   }

   public PointCloud(Point3f[] points)
   {
      addAll(points);
   }

   public PointCloud(List<Point3f> points)
   {
      addAll(points);
   }

   public int getNumberOfPoints()
   {
      return points.size();
   }

   public boolean isEmpty()
   {
      return points.isEmpty();
   }

   public void clear()
   {
      // delete the points
      if (!points.isEmpty())
      {
         points.clear();
      }
   }

   public void add(double x, double y, double z)
   {
      add((float) x, (float) y, (float) z);
   }

   public void add(float x, float y, float z)
   {
      points.add(new Point3f(x, y, z));
   }

   public void add(Point3d point)
   {
      points.add(new Point3f(point));
   }

   public void add(Point3f point)
   {
      points.add(point);
   }

   public void addAll(Iterable<Point3f> points)
   {
      for (Point3f point : points)
         add(new Point3f(point));
   }

   public void addAll(Point3f[] points)
   {
      for (Point3f point : points)
         add(point);
   }

   public void addAll(float[] points)
   {
      int numberOfPoints = points.length / 3;

      for (int i = 0; i < numberOfPoints; i++)
      {
         float x = points[3 * i];
         float y = points[3 * i + 1];
         float z = points[3 * i + 2];
         add(x, y, z);
      }
   }

   /// Apply transform to each point
   public void transform(RigidBodyTransform transform)
   {
      parallelStream().forEach(point -> transform.transform(point));
   }

   /**
    * Calculate bounding box of Pointcloud
    */
   public void calculateBoundingBox(Point3d lowerBoundToPack, Point3d upperBoundToPack)
   {
      lowerBoundToPack.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      lowerBoundToPack.set(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      for (Point3f point : this)
      {
         double x = point.getX();
         double y = point.getY();
         double z = point.getZ();

         if (x < lowerBoundToPack.getX())
            lowerBoundToPack.setX(x);
         else if (x > upperBoundToPack.getX())
            upperBoundToPack.setX(x);

         if (y < lowerBoundToPack.getY())
            lowerBoundToPack.setY(y);
         else if (y > upperBoundToPack.getY())
            upperBoundToPack.setY(y);

         if (z < lowerBoundToPack.getZ())
            lowerBoundToPack.setZ(z);
         else if (z > upperBoundToPack.getZ())
            upperBoundToPack.setZ(z);
      }
   }

   public Point3f getLast()
   {
      return points.get(points.size() - 1);
   }

   /// Returns a copy of the ith point in point cloud.
   /// Use operator[] for direct access to point reference.
   public Point3f getPoint(int i) // may return NULL
   {
      return points.get(i);
   }

   public Point3f removePoint(int i)
   {
      return points.remove(i);
   }

   public Stream<Point3f> stream()
   {
      return points.stream();
   }

   public Stream<Point3f> parallelStream()
   {
      return points.parallelStream();
   }

   @Override
   public Iterator<Point3f> iterator()
   {
      return points.iterator();
   }
}
