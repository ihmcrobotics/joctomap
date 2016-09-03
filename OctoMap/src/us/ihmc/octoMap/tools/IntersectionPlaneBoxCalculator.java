package us.ihmc.octoMap.tools;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import gnu.trove.list.array.TDoubleArrayList;
import javafx.util.Pair;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class IntersectionPlaneBoxCalculator
{
   private static final double EPSILON = 1.0e-3;

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

   private final Vector3d edgeVector = new Vector3d();
   private final Vector3d fromPlaneCenterToEdgeStart = new Vector3d();
   private final RecyclingArrayList<Point3d> unorderedIntersections = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3d.class));

   public List<Point3d> computeIntersections()
   {
      RecyclingArrayList<Point3d> intersections = new RecyclingArrayList<>(Point3d.class);
      computeIntersections(intersections);
      return intersections;
   }

   private final Point3d intersection = new Point3d();

   public void computeIntersections(RecyclingArrayList<Point3d> intersectionsToPack)
   {
      unorderedIntersections.clear();

      for (int i = 0; i < 12; i++)
      {
         Point3d edgeStart = boxEdges.get(i).getKey();
         Point3d edgeEnd = boxEdges.get(i).getValue();
         edgeVector.sub(edgeEnd, edgeStart);

         fromPlaneCenterToEdgeStart.sub(pointOnPlane, boxCenter);
         fromPlaneCenterToEdgeStart.sub(edgeStart);

         double dotNormalEdge = planeNormal.dot(edgeVector);

         if (Math.abs(dotNormalEdge) < 1.0e-5)
            continue;

         double scaleFactor = planeNormal.dot(fromPlaneCenterToEdgeStart) / dotNormalEdge;
         if (scaleFactor < 0.0 || scaleFactor > 1.0)
            continue;

         intersection.scaleAdd(scaleFactor, edgeVector, edgeStart);
         intersection.setX(intersection.getX() * boxSize.getX());
         intersection.setY(intersection.getY() * boxSize.getY());
         intersection.setZ(intersection.getZ() * boxSize.getZ());
         intersection.add(boxCenter);
         if (!listContains(unorderedIntersections, intersection))
            unorderedIntersections.add().set(intersection);

         if (unorderedIntersections.size() == 6) // That's the max number of possible intersections
            break;
      }
      reorderIntersections(unorderedIntersections, intersectionsToPack);
   }

   private final TDoubleArrayList orderedAngles = new TDoubleArrayList();
   private final Vector3d v0 = new Vector3d();
   private final Vector3d vi = new Vector3d();
   private final Vector3d vCross = new Vector3d();

   private void reorderIntersections(List<Point3d> unorderedIntersections, RecyclingArrayList<Point3d> intersectionsToPack)
   {
      intersectionsToPack.clear();
      if (unorderedIntersections.isEmpty())
         return;

      orderedAngles.reset();
      intersectionsToPack.add().set(unorderedIntersections.get(0));
      orderedAngles.add(0.0);

      v0.sub(unorderedIntersections.get(0), pointOnPlane);
      v0.normalize();


      for (int i = 1; i < unorderedIntersections.size(); i++)
      {
         vi.sub(unorderedIntersections.get(i), pointOnPlane);
         vi.normalize();

         double angle = v0.dot(vi); // Gives [1.0, -1.0] for an angle in [0, Pi]

         vCross.cross(v0, vi);
         if (vCross.dot(planeNormal) < 0.0) // Allows to make a difference between the two angle ranges: [0, Pi] and [Pi, 2*Pi]
            angle = -2 - angle; // We're in the range [Pi, 2*Pi], this transform the dot original range [1, -1] to [-1, -3] where -3 is when vi & v0 point in the same direction. 
         // We obtained an "angle" that goes from 1 down to -3. It is transformed to be in the range [0, 4].
         angle = - (angle - 1.0);

         // Binary search to figure out where the vertex should go in the list.
         int index = angleBinarySearch(orderedAngles, angle);
         if (index < 0)
            index = -index - 1;

         intersectionsToPack.insertAtIndex(index).set(unorderedIntersections.get(i));
         orderedAngles.insert(index, angle);
      }
   }

   private static int angleBinarySearch(TDoubleArrayList l, double key)
   {
      int low = 0;
      int high = l.size() - 1;

      while (low <= high)
      {
         int mid = (low + high) >>> 1;
         double midVal = l.get(mid);
         int cmp = compare(midVal, key);

         if (cmp < 0)
            low = mid + 1;
         else if (cmp > 0)
            high = mid - 1;
         else
            return mid; // key found
      }
      return -(low + 1); // key not found
   }

   private static int compare(double angle1, double angle2)
   {
      if (angle1 == angle2)
         return 0;
      else if (angle1 < angle2)
         return -1;
      // (angle1 > angle2)
      return 1;
   }

   private boolean listContains(List<Point3d> listOfPoints, Point3d pointToCheck)
   {
      for (int i = 0; i < listOfPoints.size(); i++)
      {
         if (epsilonEquals(listOfPoints.get(i), pointToCheck))
            return true;
      }
      return false;
   }

   private boolean epsilonEquals(Point3d point1, Point3d point2)
   {
      double diff;

      diff = point1.getX() - point2.getX();
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > EPSILON * boxSize.getX())
         return false;

      diff = point1.getY() - point2.getY();
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > EPSILON * boxSize.getY())
         return false;

      diff = point1.getZ() - point2.getZ();
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > EPSILON * boxSize.getZ())
         return false;

      return true;
   }
}
