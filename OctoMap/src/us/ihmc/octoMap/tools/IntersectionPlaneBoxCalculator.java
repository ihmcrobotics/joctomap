package us.ihmc.octoMap.tools;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import javafx.util.Pair;
import us.ihmc.robotics.MathTools;

public class IntersectionPlaneBoxCalculator
{
   private final Point3d[] boxVertices = new Point3d[8];
   private final List<Pair<Point3d, Point3d>> boxEdges = new ArrayList<>();

   private final Vector3d boxSize = new Vector3d();
   private final Point3d boxCenter = new Point3d();

   private final Point3d pointOnPlane = new Point3d();
   private final Vector3d planeNormal = new Vector3d();

   public IntersectionPlaneBoxCalculator()
   {
      // Index ordering as in http://paulbourke.net/geometry/polygonise/
      boxVertices[0] = new Point3d( 0.5,  0.5, -0.5);
      boxVertices[1] = new Point3d( 0.5, -0.5, -0.5);
      boxVertices[2] = new Point3d(-0.5, -0.5, -0.5);
      boxVertices[3] = new Point3d(-0.5,  0.5, -0.5);
      boxVertices[4] = new Point3d( 0.5,  0.5,  0.5);
      boxVertices[5] = new Point3d( 0.5, -0.5,  0.5);
      boxVertices[6] = new Point3d(-0.5, -0.5,  0.5);
      boxVertices[7] = new Point3d(-0.5,  0.5,  0.5);

      boxEdges.add(new Pair<>(boxVertices[0], boxVertices[1]));
      boxEdges.add(new Pair<>(boxVertices[1], boxVertices[2]));
      boxEdges.add(new Pair<>(boxVertices[2], boxVertices[3]));
      boxEdges.add(new Pair<>(boxVertices[3], boxVertices[0]));
      boxEdges.add(new Pair<>(boxVertices[4], boxVertices[5]));
      boxEdges.add(new Pair<>(boxVertices[5], boxVertices[6]));
      boxEdges.add(new Pair<>(boxVertices[6], boxVertices[7]));
      boxEdges.add(new Pair<>(boxVertices[7], boxVertices[4]));
      boxEdges.add(new Pair<>(boxVertices[0], boxVertices[4]));
      boxEdges.add(new Pair<>(boxVertices[1], boxVertices[5]));
      boxEdges.add(new Pair<>(boxVertices[2], boxVertices[6]));
      boxEdges.add(new Pair<>(boxVertices[3], boxVertices[7]));
   }

   public void setCube(double size, Point3d center)
   {
      setBox(size, size, size, center);
   }

   public void setBox(double lx, double ly, double lz, Point3d center)
   {
      boxSize.set(lx, ly, lz);
      boxCenter.set(center);

   }

   public void setPlane(Point3d pointOnPlane, Vector3d planeNormal)
   {
      this.pointOnPlane.set(pointOnPlane);
      this.planeNormal.set(planeNormal);
   }

   public void computeIntersections(List<Point3d> intersectionsToPack)
   {
      Vector3d edgeVector = new Vector3d();
      intersectionsToPack.clear();
      
      int count = 0;
      
      for (int i = 0; i < 12; i++)
      {
         Point3d intersection = new Point3d();
         Point3d edgeStart = boxEdges.get(i).getKey();
         Point3d edgeEnd = boxEdges.get(i).getValue();
         edgeVector.sub(edgeEnd, edgeStart);

         Vector3d fromPlaneCenterToEdgeStart = new Vector3d();
         fromPlaneCenterToEdgeStart.sub(pointOnPlane, boxCenter);
         fromPlaneCenterToEdgeStart.sub(edgeStart);

         double dotNormalEdge = planeNormal.dot(edgeVector);

         if (Math.abs(dotNormalEdge) < 1.0e-5)
            continue;

         double scaleFactor = planeNormal.dot(fromPlaneCenterToEdgeStart) / dotNormalEdge;
         if (!MathTools.isInsideBoundsInclusive(scaleFactor, 0.0, 1.0))
            continue;

         intersection.scaleAdd(scaleFactor, edgeVector, edgeStart);
         intersection.x *= boxSize.x;
         intersection.y *= boxSize.y;
         intersection.z *= boxSize.z;
         intersection.add(boxCenter);
         intersectionsToPack.add(intersection);
         count++;

         if (count == 6) // That's the max number of possible intersections
            break;
      }
      reorderIntersections(intersectionsToPack);
   }

   private void reorderIntersections(List<Point3d> intersectionsToPack)
   {
      if (intersectionsToPack.isEmpty())
         return;

      final Vector3d v1 = new Vector3d();
      final Vector3d v2 = new Vector3d();
      final Vector3d v3 = new Vector3d();

      Collections.sort(intersectionsToPack, new Comparator<Point3d>()
      {
         @Override
         public int compare(Point3d o1, Point3d o2)
         {
            if (o1.epsilonEquals(o2, 1.0e-10))
               return 0;
            v1.sub(o1, pointOnPlane);
            v2.sub(o2, pointOnPlane);
            v3.cross(v1, v2);
            return v3.dot(planeNormal) < 0.0 ? 1 : -1;
         }
      });
   }
}
