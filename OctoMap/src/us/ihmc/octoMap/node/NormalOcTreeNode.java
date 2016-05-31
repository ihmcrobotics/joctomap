package us.ihmc.octoMap.node;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class NormalOcTreeNode extends AbstractOccupancyOcTreeNode<NormalOcTreeNode>
{
   private Vector3d normal = null;
   private List<Point3d> plane = null;

   public NormalOcTreeNode()
   {
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
            x += childNormal.x;
            y += childNormal.y;
            z += childNormal.z;
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
      this.normal = normal;
   }

   public boolean isNormalSet()
   {
      return normal != null;
   }
   
   public List<Point3d> getPlane()
   {
      return plane;
   }
   
   public void setPlane(List<Point3d> plane)
   {
      this.plane = plane;
   }
}
