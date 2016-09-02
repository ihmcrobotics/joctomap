package us.ihmc.octoMap.node;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.ocTree.PlanarRegion;

public class NormalOcTreeNode extends AbstractOccupancyOcTreeNode<NormalOcTreeNode>
{
   private Vector3d normal = null;
   private List<Point3d> plane = null;
   private int regionId = PlanarRegion.NO_REGION_ID;

   public NormalOcTreeNode()
   {
   }

   @Override
   public void copyData(NormalOcTreeNode other)
   {
      super.copyData(other);
      if (other.normal != null)
      {
         if (normal == null)
            normal = new Vector3d();
         normal.set(other.normal);
      }
      if (other.plane != null)
      {
         if (plane == null)
            plane = new ArrayList<>();
         while (plane.size() < other.plane.size())
            plane.add(new Point3d());
         while (plane.size() > other.plane.size())
            plane.remove(plane.size() - 1);
         for (int i = 0; i < other.plane.size(); i++)
            plane.get(i).set(other.plane.get(i));
      }
   }

   @Override
   public void allocateChildren()
   {
      children = new NormalOcTreeNode[8];
   }

   @Override
   public NormalOcTreeNode create()
   {
      return new NormalOcTreeNode();
   }

   public void resetRegionId()
   {
      regionId = PlanarRegion.NO_REGION_ID;
   }

   public void updateNormalChildren()
   {
      setNormal(getAverageChildNormal());
   }

   public Vector3d getAverageChildNormal()
   {
      if (children == null)
         return null;

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      int count = 0;

      for (int i = 0; i < 8; i++)
      {
         NormalOcTreeNode child = children[i];

         if (child != null && child.isNormalSet())
         {
            Vector3d childNormal = child.getNormal();
            x += childNormal.getX();
            y += childNormal.getY();
            z += childNormal.getZ();
            count++;
         }
      }

      if (count == 0)
         return null;
      x /= count;
      y /= count;
      z /= count;
      Vector3d normal = new Vector3d(x, y, z);
      normal.normalize();
      return normal;
   }

   public Vector3d getNormal()
   {
      return normal;
   }

   public void setNormal(Vector3d normal)
   {
      if (this.normal == null)
         this.normal = new Vector3d();
      this.normal.set(normal);
   }

   public void resetNormal()
   {
      if (normal != null)
         normal.set(Double.NaN, Double.NaN, Double.NaN);
   }

   public boolean isNormalSet()
   {
      return normal != null && !Double.isNaN(normal.getX()) && !Double.isNaN(normal.getY()) && !Double.isNaN(normal.getZ());
   }
   
   public List<Point3d> getPlane()
   {
      return plane;
   }
   
   public void setPlane(List<Point3d> plane)
   {
      this.plane = plane;
   }

   public boolean isPartOfRegion()
   {
      return regionId != PlanarRegion.NO_REGION_ID;
   }

   public void setRegionId(int regionId)
   {
      this.regionId = regionId;
   }

   public int getRegionId()
   {
      return regionId;
   }
}
