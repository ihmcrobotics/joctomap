package us.ihmc.octoMap.planarRegions;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.NormalOcTreeNode;

public class PlanarRegion
{
   public static final int NO_REGION_ID = Integer.MIN_VALUE;

   private int id = NO_REGION_ID;

   private final VectorMean normal = new VectorMean();
   private final PointMean point = new PointMean();
   private final Vector3d temporaryVector = new Vector3d();
   private final List<Point3d> points = new ArrayList<>();
   private final List<OcTreeKey> keys = new ArrayList<>();

   public PlanarRegion(int id)
   {
      this.id = id;
   }

   public void addPoint(Point3d point)
   {
      points.add(point);
   }

   public void update(NormalOcTreeNode node, OcTreeKeyReadOnly nodeKey)
   {
      node.getNormal(temporaryVector);
      // TODO Review and possibly improve dealing with normal flips.
      if (getNumberOfNodes() >= 1 && temporaryVector.dot(normal) < 0.0)
         temporaryVector.negate();
      normal.update(temporaryVector);

      Point3d newPoint = new Point3d();
      node.getHitLocation(newPoint);
      points.add(newPoint);
      point.update(newPoint);

      keys.add(new OcTreeKey(nodeKey));
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

   public Vector3d getNormal()
   {
      return normal;
   }

   public Point3d getOrigin()
   {
      return point;
   }

   public Point3d getPoint(int index)
   {
      return points.get(index);
   }
 
   public List<Point3d> getPoints()
   {
      return points;
   }

   public int getId()
   {
      return id;
   }

   public int getNumberOfNodes()
   {
      return points.size();
   }

   public void printPointsToFile()
   {
      try
      {
         FileWriter fw = new FileWriter("regionPoints");
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

   @Override
   public String toString()
   {
      String ret = "Region ID: " + id;
      ret += ", origin: " + point + ", normal: " + normal;
      ret += ", size: " + points.size();
      return ret;
   }
}
