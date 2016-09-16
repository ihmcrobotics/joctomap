package us.ihmc.octoMap.pointCloud;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

public class SweepCollection
{
   private final List<PointCloud> sweeps = new ArrayList<>();
   private final List<Point3d> origins = new ArrayList<>();

   private int subSampleSize = 10000;

   public SweepCollection()
   {
   }

   public SweepCollection(PointCloud pointCloud, Point3d origin)
   {
      addSweep(pointCloud, origin);
   }

   public void clear()
   {
      sweeps.clear();
      origins.clear();
   }

   public void setSubSampleSize(int subSampleSize)
   {
      this.subSampleSize = subSampleSize;
   }

   public void addSweepCollection(SweepCollection sweepCollection)
   {
      for (int i = 0; i < sweepCollection.getNumberOfSweeps(); i++)
         addSweep(sweepCollection.getSweep(i), sweepCollection.getSweepOrigin(i));
   }

   public void addSweep(PointCloud pointCloud, Point3d origin)
   {
      sweeps.add(pointCloud);
      origins.add(origin);
   }

   public void addSweep(float[] pointsInWorld, Point3d origin)
   {
      sweeps.add(PointCloud.createPointCloudFromSubSample(pointsInWorld, subSampleSize));
      origins.add(new Point3d(origin));
   }

   public void addSweep(float[] pointsInWorld, Point3f origin)
   {
      sweeps.add(PointCloud.createPointCloudFromSubSample(pointsInWorld, subSampleSize));
      origins.add(new Point3d(origin));
   }

   public PointCloud getSweep(int index)
   {
      return sweeps.get(index);
   }

   public Point3d getSweepOrigin(int index)
   {
      return origins.get(index);
   }

   public int getNumberOfSweeps()
   {
      return sweeps.size();
   }
}
