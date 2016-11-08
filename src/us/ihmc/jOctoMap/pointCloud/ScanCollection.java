package us.ihmc.jOctoMap.pointCloud;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.jOctoMap.tools.PointCloudTools;

public class ScanCollection implements Iterable<Scan>
{
   private final List<Scan> scans = new ArrayList<>();

   private int subSampleSize = 10000;

   public ScanCollection()
   {
   }

   public ScanCollection(PointCloud pointCloud, Point3d origin)
   {
      addScan(pointCloud, origin);
   }

   public void clear()
   {
      scans.clear();
   }

   public void setSubSampleSize(int subSampleSize)
   {
      this.subSampleSize = subSampleSize;
   }

   public void addSweepCollection(ScanCollection sweepCollection)
   {
      for (int i = 0; i < sweepCollection.getNumberOfScans(); i++)
         addScan(sweepCollection.getScan(i));
   }

   public void addScan(Scan scan)
   {
      scans.add(scan);
   }

   public void addScan(PointCloud pointCloud, Point3d origin)
   {
      scans.add(new Scan(origin, pointCloud));
   }

   public void addScan(float[] pointsInWorld, Point3d origin)
   {
      scans.add(new Scan(origin, PointCloudTools.createRandomSample(pointsInWorld, subSampleSize)));
   }

   public void addScan(float[] pointsInWorld, Point3f origin)
   {
      scans.add(new Scan(new Point3d(origin), PointCloudTools.createRandomSample(pointsInWorld, subSampleSize)));
   }

   public Scan getScan(int index)
   {
      return scans.get(index);
   }

   public int getNumberOfScans()
   {
      return scans.size();
   }

   public int getNumberOfPoints()
   {
      int numberOfPoints = 0;
      for (Scan scan : scans)
         numberOfPoints += scan.getNumberOfPoints();
      return numberOfPoints;
   }

   @Override
   public Iterator<Scan> iterator()
   {
      return scans.iterator();
   }
}
