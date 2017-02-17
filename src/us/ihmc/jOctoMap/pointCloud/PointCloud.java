package us.ihmc.jOctoMap.pointCloud;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Stream;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class PointCloud implements Iterable<Point3D32>
{
   protected final List<Point3D32> points = new ArrayList<>();

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

   public PointCloud(Point3DReadOnly[] points)
   {
      addAll(points);
   }

   public PointCloud(List<? extends Point3DReadOnly> points)
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
      points.add(new Point3D32(x, y, z));
   }

   public void add(Point3DReadOnly point)
   {
      points.add(new Point3D32(point));
   }

   public void addAll(Iterable<? extends Point3DReadOnly> points)
   {
      for (Point3DReadOnly point : points)
         add(new Point3D32(point));
   }

   public void addAll(Point3DReadOnly[] points)
   {
      for (Point3DReadOnly point : points)
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
   public void transform(Transform transform)
   {
      points.parallelStream().forEach(point -> transform.transform(point));
   }

   /**
    * Calculate bounding box of Pointcloud
    */
   public void calculateBoundingBox(Point3DBasics lowerBoundToPack, Point3DBasics upperBoundToPack)
   {
      lowerBoundToPack.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      lowerBoundToPack.set(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      for (Point3DReadOnly point : this)
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

   public Point3DReadOnly getLast()
   {
      return points.get(points.size() - 1);
   }

   /// Returns a copy of the ith point in point cloud.
   /// Use operator[] for direct access to point reference.
   public Point3DReadOnly getPoint(int i) // may return NULL
   {
      return points.get(i);
   }

   public Point3D32 removePoint(int i)
   {
      return points.remove(i);
   }

   public Stream<? extends Point3DReadOnly> stream()
   {
      return points.stream();
   }

   public Stream<? extends Point3DReadOnly> parallelStream()
   {
      return points.parallelStream();
   }

   @Override
   public Iterator<Point3D32> iterator()
   {
      return points.iterator();
   }
}
