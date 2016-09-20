package us.ihmc.octoMap.planarRegions;

import java.io.FileWriter;
import java.io.IOException;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.lists.RecyclingArrayList;

public class PlanarRegion
{
   public static final int NO_REGION_ID = Integer.MIN_VALUE;

   private int id = NO_REGION_ID;

   private final VectorMean normal = new VectorMean();
   private final PointMean point = new PointMean();
   private final Vector3d temporaryVector = new Vector3d();
   private final RecyclingArrayList<Point3d> points = new RecyclingArrayList<>(Point3d.class);

   public PlanarRegion(int id)
   {
      this.id = id;
   }

   public void update(Vector3d normal, Point3d point)
   {
      this.normal.update(normal);
      this.point.update(point);
      points.add().set(point);
   }

   public double distanceFromCenterSquared(Point3d point)
   {
      return this.point.distanceSquared(point);
   }

   public double orthogonalDistance(Point3d point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.point);
      return temporaryVector.dot(this.normal);
   }

   public double absoluteOrthogonalDistance(Point3d point)
   {
      return Math.abs(orthogonalDistance(point));
   }

   public double angle(Vector3d normal)
   {
      return this.normal.angle(normal);
   }

   public double absoluteAngle(Vector3d normal)
   {
      return Math.abs(angle(normal));
   }

   public double dot(Vector3d normal)
   {
      return this.normal.dot(normal);
   }

   public double absoluteDot(Vector3d normal)
   {
      return Math.abs(dot(normal));
   }

   public int getId()
   {
      return id;
   }

   public int getNumberOfNodes()
   {
      return normal.getNumberOfSamples();
   }

   public void printPointsToFile()
   {
      try
      {
         FileWriter fw = new FileWriter("regionPoints" + id);
         for (int i = 0; i < getNumberOfNodes(); i++)
         {
            Point3d point = points.get(i);
            fw.write(point.x + ", " + point.y + ", " + point.z + "\n");
         }
         fw.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
