package us.ihmc.octoMap.tools;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.node.AbstractOcTreeNode;

public class OcTreeTools
{
   /**
    * Return centers of leafs that do NOT exist (but could) in a given bounding box
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> List<Point3d> getUnknownLeafCenters(NODE root, Point3d pmin, Point3d pmax, double resolution,
         int treeDepth)
   {
      return getUnknownLeafCenters(root, pmin, pmax, 0, resolution, treeDepth);
   }

   /**
    * Return centers of leafs that do NOT exist (but could) in a given bounding box
    */
   public static <NODE extends AbstractOcTreeNode<NODE>> List<Point3d> getUnknownLeafCenters(NODE root, Point3d pmin, Point3d pmax, int depth,
         double resolution, int treeDepth)
   {
      OctoMapTools.checkIfDepthValid(depth, treeDepth);
      if (depth == 0)
         depth = treeDepth;

      List<Point3d> nodeCenters = new ArrayList<>();

      double[] pminArray = new double[3];
      double[] pmaxArray = new double[3];
      pmin.get(pminArray);
      pmax.get(pmaxArray);

      double[] diff = new double[3];
      int[] steps = new int[3];
      double stepSize = resolution * Math.pow(2, treeDepth - depth);
      for (int i = 0; i < 3; ++i)
      {
         diff[i] = pmaxArray[i] - pminArray[i];
         steps[i] = (int) Math.floor(diff[i] / stepSize);
         //      std::cout << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
      }

      Point3d p = new Point3d(pmin);
      NODE res;
      for (int x = 0; x < steps[0]; ++x)
      {
         p.setX(p.getX() + stepSize);
         for (int y = 0; y < steps[1]; ++y)
         {
            p.setY(p.getY() + stepSize);
            for (int z = 0; z < steps[2]; ++z)
            {
               //          std::cout << "querying p=" << p << std::endl;
               p.setZ(p.getZ() + stepSize);
               res = OcTreeSearchTools.search(root, p, depth, resolution, treeDepth);
               if (res == null)
               {
                  nodeCenters.add(p);
               }
            }
            p.setZ(pmin.getZ());
         }
         p.setY(pmin.getY());
      }
      return nodeCenters;
   }
}
