package us.ihmc.jOctoMap.pointCloud;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.tools.PointCloudTools;

public class ScanCollection implements Iterable<Scan>
{
   private final List<Scan> scans = new ArrayList<>();

   private int subSampleSize = 10000;

   public ScanCollection()
   {
   }

   public ScanCollection(PointCloud pointCloud, Point3DReadOnly origin)
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

   public void addScan(PointCloud pointCloud, Point3DReadOnly origin)
   {
      scans.add(new Scan(origin, pointCloud));
   }

   public void addScan(float[] pointsInWorld, Point3DReadOnly origin)
   {
      scans.add(new Scan(origin, PointCloudTools.createRandomSample(pointsInWorld, subSampleSize)));
   }

   public void transform(Transform transform)
   {
      scans.stream().forEach(scan -> scan.transform(transform));
   }

   public Scan getScan(int index)
   {
      return scans.get(index);
   }

   public Scan getLastScan()
   {
      if (scans.isEmpty())
         return null;
      else
         return scans.get(scans.size() - 1);
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
