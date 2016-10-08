package us.ihmc.octoMap.planarRegions;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.node.NormalOcTreeNode;

public class PlanarRegion
{
   public static final int NO_REGION_ID = Integer.MIN_VALUE;

   private int id = NO_REGION_ID;

   private final VectorMean normal = new VectorMean();
   private final PointMean point = new PointMean();
   private final Vector3d temporaryVector = new Vector3d();

   private final List<NormalOcTreeNode> nodes = new ArrayList<>();

   public PlanarRegion(int id)
   {
      this.id = id;
   }

   public void addNode(NormalOcTreeNode node)
   {
      updateNormalAndOriginOnly(node);
      node.setRegionId(id);
      nodes.add(node);
   }

   public void merge(PlanarRegion other)
   {
      other.nodeStream().forEach(this::addNode);
   }

   public void recomputeNormalAndOrigin()
   {
      point.clear();
      normal.clear();
      nodes.stream().forEach(node -> updateNormalAndOriginOnly(node));
   }

   private void updateNormalAndOriginOnly(NormalOcTreeNode node)
   {
      node.getNormal(temporaryVector);
      // TODO Review and possibly improve dealing with normal flips.
      if (getNumberOfNodes() >= 1 && temporaryVector.dot(normal) < 0.0)
         temporaryVector.negate();
      normal.update(temporaryVector);
      point.update(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ());
   }

   public double distanceFromCenterSquared(Point3d point)
   {
      return this.point.distanceSquared(point);
   }

   public double orthogonalDistance(Point3d point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.point);
      return temporaryVector.dot(normal);
   }

   public double absoluteOrthogonalDistance(Point3d point)
   {
      return Math.abs(orthogonalDistance(point));
   }

   public double orhtogonalDistance(NormalOcTreeNode node)
   {
      temporaryVector.set(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ());
      temporaryVector.sub(point);
      return temporaryVector.dot(normal);
   }

   public double absoluteOrthogonalDistance(NormalOcTreeNode node)
   {
      return Math.abs(orhtogonalDistance(node));
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

   public double dot(PlanarRegion other)
   {
      return dot(other.normal);
   }

   public double absoluteDot(PlanarRegion other)
   {
      return Math.abs(dot(other));
   }

   public double dotWithNodeNormal(NormalOcTreeNode node)
   {
      return normal.getX() * node.getNormalX() + normal.getY() * node.getNormalY() + normal.getZ() * node.getNormalZ();
   }

   public double absoluteDotWithNodeNormal(NormalOcTreeNode node)
   {
      return Math.abs(dotWithNodeNormal(node));
   }

   public Vector3d getNormal()
   {
      return normal;
   }

   public Point3d getOrigin()
   {
      return point;
   }

   public void getPoint(int index, Point3d pointToPack)
   {
      nodes.get(index).getHitLocation(pointToPack);
   }

   public int getId()
   {
      return id;
   }

   public NormalOcTreeNode getNode(int index)
   {
      return nodes.get(index);
   }

   public void removeNode(int index)
   {
      nodes.remove(index);
   }

   public int getNumberOfNodes()
   {
      return nodes.size();
   }

   public Stream<NormalOcTreeNode> nodeStream()
   {
      return nodes.stream();
   }

   public void printPointsToFile()
   {
      try
      {
         FileWriter fw = new FileWriter("regionPoints");
         for (int i = 0; i < getNumberOfNodes(); i++)
         {
            NormalOcTreeNode point = nodes.get(i);
            fw.write(point.getHitLocationX() + ", " + point.getHitLocationY() + ", " + point.getHitLocationZ() + "\n");
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
      ret += ", size: " + getNumberOfNodes();
      return ret;
   }
}
