package us.ihmc.jOctoMap.tools;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;

public abstract class OcTreeTools
{
   public static <NODE extends AbstractOcTreeNode<NODE>> void computeMinMaxCoordinates(NODE root, Point3d minToPack, Point3d maxToPack, double resolution, int treeDepth)
   {
      // empty tree
      if (root == null)
      {
         minToPack.set(0.0, 0.0, 0.0);
         maxToPack.set(0.0, 0.0, 0.0);
         return;
      }

      minToPack.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      maxToPack.set(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      for (NODE node : OcTreeIteratorFactory.createLeafIterable(root))
      {
         double size = node.getSize();
         double halfSize = size / 2.0;
         double x = node.getX() - halfSize;
         double y = node.getY() - halfSize;
         double z = node.getZ() - halfSize;

         minToPack.setX(Math.min(x, minToPack.getX()));
         minToPack.setY(Math.min(y, minToPack.getY()));
         minToPack.setZ(Math.min(z, minToPack.getZ()));

         x += size;
         y += size;
         z += size;

         maxToPack.setX(Math.max(x, maxToPack.getX()));
         maxToPack.setY(Math.max(y, maxToPack.getY()));
         maxToPack.setZ(Math.max(z, maxToPack.getZ()));
      }
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> Point3d computeMaxCoordinate(NODE root, double resolution, int treeDepth)
   {
      // empty tree
      if (root == null)
         return new Point3d();

      Point3d max = new Point3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      for (NODE node : OcTreeIteratorFactory.createLeafIterable(root))
      {
         double size = node.getSize();
         double halfSize = size / 2.0;
         double x = node.getX() + halfSize;
         double y = node.getY() + halfSize;
         double z = node.getZ() + halfSize;

         max.setX(Math.max(x, max.getX()));
         max.setY(Math.max(y, max.getY()));
         max.setZ(Math.max(z, max.getZ()));
      }
      return max;
   }

   public static <NODE extends AbstractOcTreeNode<NODE>> Point3d computeMinCoordinate(NODE root, double resolution, int treeDepth)
   {
      // empty tree
      if (root == null)
         return new Point3d();

      Point3d min = new Point3d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

      for (NODE node : OcTreeIteratorFactory.createLeafIterable(root))
      {
         double size = node.getSize();
         double halfSize = size / 2.0;
         double x = node.getX() - halfSize;
         double y = node.getY() - halfSize;
         double z = node.getZ() - halfSize;

         min.setX(Math.min(x, min.getX()));
         min.setY(Math.min(y, min.getY()));
         min.setZ(Math.min(z, min.getZ()));
      }
      return min;
   }

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
      JOctoMapTools.checkIfDepthValid(depth, treeDepth);
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
