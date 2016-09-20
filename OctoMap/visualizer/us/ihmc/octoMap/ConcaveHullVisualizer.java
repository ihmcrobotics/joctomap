package us.ihmc.octoMap;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.opensphere.geometry.algorithm.ConcaveHull;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.MultiPoint;

import javafx.application.Application;
import javafx.event.Event;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.PointLight;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.robotics.time.TimeTools;

public class ConcaveHullVisualizer extends Application
{
   private final List<Point2d> pointCloud = new ArrayList<>();
   private final List<Point2d> concaveHullVertices = new ArrayList<>();
   private final MultiColorMeshBuilder meshBuilder = new MultiColorMeshBuilder();
   private ConvexPolygon2d convexPolygon2d;

   public ConcaveHullVisualizer() throws IOException
   {
      InputStreamReader inputStreamReader = new InputStreamReader(getClass().getResourceAsStream("regionPoints"));
      BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

      String line = "";
      String cvsSplitBy = ",";

      while ((line = bufferedReader.readLine()) != null)
      {
         String[] coordsAsString = line.split(cvsSplitBy);
         double x = Double.parseDouble(coordsAsString[0]);
         double y = Double.parseDouble(coordsAsString[1]);
         pointCloud.add(new Point2d(x, y));
      }

      long startTime = System.nanoTime();

      ConcaveHull concaveHull = new ConcaveHull(convertPoint2dToMultipoint(pointCloud), 0.05);
      Geometry concaveHullGeometry = concaveHull.getConcaveHull();

      concaveHullVertices.addAll(getGeometryVertices(concaveHullGeometry));
      
      ensureClockwiseOrdering();
      
      System.out.println("Size before filtering: " + concaveHullVertices.size());

      long endTime = System.nanoTime();
      System.out.println("ConcaveHull Took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

      System.out.println("Perimeter: " + computeTotalLength());
      double sd = computeMaxStandardDeviation();
      System.out.println("Standard dev: " + sd);


      for (int i = 0; i < 3; i++)
      {
         filter2(sd / 5.0);
         filterOutPeaks();
         filterOutShortEdges(sd / 10.0);
//         filterOutSmallTriangles();
      }

      convexPolygon2d = new ConvexPolygon2d(concaveHullVertices);

      startTime = System.nanoTime();
      double depthThreshold = 0.1;
      decomposeRecursively(concaveHullVertices, 0, concaveHullVertices.size(), depthThreshold, 0, decomposedPolygons);
      endTime = System.nanoTime();
      System.out.println("decomposition Took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime) + ", number of polygons: " + decomposedPolygons.size());
      
      System.out.println("Size after filtering: " + concaveHullVertices.size());
   }

   private final List<ConvexPolygon2d> decomposedPolygons = new ArrayList<>();

   private void decomposeRecursively(List<Point2d> concaveHullVertices, int startIndex, int endIndex, double depthThreshold, int decompositionDepth, List<ConvexPolygon2d> convexPolygons)
   {
      if (concaveHullVertices.isEmpty())
         throw new RuntimeException("The concave hull is empty");

      concaveHullVertices = subList(concaveHullVertices, startIndex, endIndex);
      ConvexPolygon2d convexHull = new ConvexPolygon2d(concaveHullVertices);

      // The concave hull is actually convex, end of recursion
      if (convexHull.getNumberOfVertices() == concaveHullVertices.size())// || decompositionDepth == 2 || convexPolygons.size() == 4)
      {
         convexPolygons.add(convexHull);
         return;
      }

//      if 

      // Find first common vertex between the two hulls. 
      int convexStartIndex = -1;
      int concaveStartIndex = -1;

      for (int i = 0; i < convexHull.getNumberOfVertices(); i++)
      {
         Point2d currentConvexVertex = convexHull.getVertex(i);

         for (int j = 0; j < concaveHullVertices.size(); j++)
         {
            Point2d currentConcaveVertex = concaveHullVertices.get(j);

            if (currentConcaveVertex.epsilonEquals(currentConvexVertex, 1.0e-7))
            {
               convexStartIndex = i;
               concaveStartIndex = j;
               break;
            }
         }

         if (convexStartIndex != -1)
            break;
      }

      if (convexStartIndex == -1 || concaveStartIndex == -1)
         throw new RuntimeException("Something went wrong finding start indices");

      // Find the first index at which a bridge starts
      int startBridgeConcaveIndex = -1;
      int startBridgeConvexIndex = -1;

      for (int indexOffset = 1; indexOffset < concaveHullVertices.size(); indexOffset++)
      {
         int currentConvexIndex = (convexStartIndex + indexOffset) % convexHull.getNumberOfVertices();
         int currentConcaveIndex = (concaveStartIndex + indexOffset) % concaveHullVertices.size();
         Point2d currentConvexVertex = convexHull.getVertex(currentConvexIndex);
         Point2d currentConcaveVertex = concaveHullVertices.get(currentConcaveIndex);

         if (!currentConvexVertex.epsilonEquals(currentConcaveVertex, 1.0e-7))
         {
            startBridgeConvexIndex = currentConvexIndex - 1;
            if (startBridgeConvexIndex == -1) startBridgeConvexIndex = convexHull.getNumberOfVertices() - 1;
            startBridgeConcaveIndex = currentConcaveIndex - 1;
            if (startBridgeConcaveIndex == -1) startBridgeConcaveIndex = concaveHullVertices.size() - 1;
            break;
         }
      }

      if (startBridgeConvexIndex == -1 || startBridgeConcaveIndex == -1)
         throw new RuntimeException("Did not find bridge.");

      // Find the deepest vertex in the pocket
      int endBridgeConcaveIndex = -1;

      Point2d firstBridgeVertex = convexHull.getVertex(startBridgeConvexIndex);
      Point2d secondBridgeVertex = convexHull.getNextVertex(startBridgeConvexIndex);
      LineSegment2d bridgeSegment = new LineSegment2d(firstBridgeVertex, secondBridgeVertex);

      int currentConcaveIndex = (startBridgeConcaveIndex + 1) % concaveHullVertices.size();
      Point2d currentConcaveVertex = concaveHullVertices.get(currentConcaveIndex);

      int deepestPocketVertexIndex = -1;
      double pocketMaxDepth = 0.0;

      while (!secondBridgeVertex.epsilonEquals(currentConcaveVertex, 1.0e-7))
      {
         double currentDepth = bridgeSegment.distance(currentConcaveVertex);

         if (currentDepth > pocketMaxDepth)
         {
            deepestPocketVertexIndex = currentConcaveIndex;
            pocketMaxDepth = currentDepth;
         }

         currentConcaveIndex = (currentConcaveIndex + 1) % concaveHullVertices.size();
         currentConcaveVertex = concaveHullVertices.get(currentConcaveIndex);
      }

      endBridgeConcaveIndex = currentConcaveIndex;

      // The pocket is negligible, remove the vertices.
      if (pocketMaxDepth < depthThreshold)
      {
         int bridgeLength;
         if (endBridgeConcaveIndex > startBridgeConcaveIndex)
            bridgeLength = endBridgeConcaveIndex - startBridgeConcaveIndex - 1;
         else
            bridgeLength = endBridgeConcaveIndex + concaveHullVertices.size() - startBridgeConcaveIndex - 1;
         for (int i = 0; i < bridgeLength; i++)
            concaveHullVertices.remove((startBridgeConcaveIndex + 1) % concaveHullVertices.size());
         // Restart the search for pockets
         decomposeRecursively(concaveHullVertices, 0, concaveHullVertices.size(), depthThreshold, decompositionDepth, convexPolygons);
         return;
      }

      Vector2d previousConcaveEdgeDirection = new Vector2d();
      Vector2d nextConcaveEdgeDirection = new Vector2d();

      previousConcaveEdgeDirection.sub(concaveHullVertices.get(deepestPocketVertexIndex), getWrap(concaveHullVertices, deepestPocketVertexIndex - 1));
      previousConcaveEdgeDirection.normalize();
      nextConcaveEdgeDirection.sub(getWrap(concaveHullVertices, deepestPocketVertexIndex + 1), concaveHullVertices.get(deepestPocketVertexIndex));
      nextConcaveEdgeDirection.normalize();

      Vector2d cutDirection = new Vector2d();
      cutDirection.interpolate(previousConcaveEdgeDirection, nextConcaveEdgeDirection, 0.5);
      cutDirection.set(cutDirection.y, -cutDirection.x); // Rotate 90 degrees to the right (inside polygon)

      Line2d cuttingLine = new Line2d(concaveHullVertices.get(deepestPocketVertexIndex), cutDirection);
      LineSegment2d edge = new LineSegment2d();

      int otherVertexIndexForCutting = -1;

      for (int currentIndex = endBridgeConcaveIndex; currentIndex < endBridgeConcaveIndex + concaveHullVertices.size(); currentIndex++)
      {
         int nextIndex = (currentIndex + 1) % concaveHullVertices.size();

         Point2d current = getWrap(concaveHullVertices, currentIndex);
         Point2d next = getWrap(concaveHullVertices, nextIndex);

         edge.set(current, next);
         Point2d intersection = edge.intersectionWith(cuttingLine);
         if (intersection != null)
         {
            concaveHullVertices.add(nextIndex, intersection);
            otherVertexIndexForCutting = nextIndex;
            if (nextIndex < deepestPocketVertexIndex) deepestPocketVertexIndex++;
//            double alpha = edge.percentageAlongLineSegment(intersection);
//            if (nextIndex == startBridgeConcaveIndex || alpha < 0.5)
//               otherVertexIndexForCutting = currentIndex % concaveHullVertices.size();
//            else
//               otherVertexIndexForCutting = nextIndex % concaveHullVertices.size();
            break;
         }
      }

      // decompose the two new polygons.
      int p1StartIndex = deepestPocketVertexIndex;
      int p1EndIndex = (otherVertexIndexForCutting + 1) % (concaveHullVertices.size() + 1);
      int p2StartIndex = otherVertexIndexForCutting;
      int p2EndIndex = (deepestPocketVertexIndex + 1) % (concaveHullVertices.size() + 1);

      int p1Size;
      if (p1StartIndex > p1EndIndex)
         p1Size = p1EndIndex + concaveHullVertices.size() - p1StartIndex;
      else
         p1Size = p1EndIndex - p1StartIndex;
      int p2Size;
      if (p2StartIndex > p2EndIndex)
         p2Size = p2EndIndex + concaveHullVertices.size() - p2StartIndex;
      else
         p2Size = p2EndIndex - p2StartIndex;
         
      if (p1Size == concaveHullVertices.size() || p2Size == concaveHullVertices.size())
         throw new RuntimeException("Something went wrong.");

      decomposeRecursively(concaveHullVertices, p1StartIndex, p1EndIndex, depthThreshold, decompositionDepth+1, convexPolygons);
      decomposeRecursively(concaveHullVertices, p2StartIndex, p2EndIndex, depthThreshold, decompositionDepth+1, convexPolygons);
   }

   private List<Point2d> subList(List<Point2d> input, int startIndex, int endIndex)
   {
      List<Point2d> output = new ArrayList<>();

      int outputLenth;
      if (endIndex > startIndex)
         outputLenth = endIndex - startIndex;
      else
         outputLenth = (endIndex + input.size()) - startIndex;
      
      int i = startIndex;
      while (output.size() != outputLenth)
      {
         output.add(getWrap(input, i));
         i++;
      }

      return output;
   }

   private Point2d getWrap(List<Point2d> list, int index)
   {
      if (index == -1)
         return list.get(list.size() - 1);
      return list.get(index % list.size());
   }

   private void ensureClockwiseOrdering()
   {
      Point2d average = new Point2d();
      double sumOfCrosses = 0.0;

      for (Point2d vertex : concaveHullVertices)
      {
         average.add(vertex);
      }
      average.scale(1.0 / concaveHullVertices.size());

      Vector2d v0 = new Vector2d();
      Vector2d v1 = new Vector2d();

      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         Point2d vertex = concaveHullVertices.get(i);
         Point2d nextVertex = concaveHullVertices.get((i + 1) % concaveHullVertices.size());
         v0.sub(vertex, average);
         v1.sub(nextVertex, average);
         sumOfCrosses += cross(v0, v1);
      }

      if (sumOfCrosses > 0.0)
      {
         Collections.reverse(concaveHullVertices);
      }
   }

   private double computeTotalLength()
   {
      double totalLength = 0.0;
      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         Point2d vertex = concaveHullVertices.get(i);
         Point2d nextVertex = concaveHullVertices.get((i + 1) % concaveHullVertices.size());
         totalLength += vertex.distance(nextVertex);
      }
      return totalLength;
   }

   private double computeMaxStandardDeviation()
   {
      long startTime = System.nanoTime();
      PrincipalComponentAnalysis3D principalComponentAnalysis3D = new PrincipalComponentAnalysis3D();
      principalComponentAnalysis3D.setPointCloud(toPoint3d(pointCloud));
      principalComponentAnalysis3D.compute();
      Vector3d standardDeviation = new Vector3d();
      principalComponentAnalysis3D.getStandardDeviation(standardDeviation);
      long endTime = System.nanoTime();
      System.out.println("Performed PCA in: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));
      return standardDeviation.length();
   }

   private List<Point3d> toPoint3d(List<Point2d> point2ds)
   {
      return toPoint3d(point2ds, 0.0);
   }

   private List<Point3d> toPoint3d(List<Point2d> point2ds, double z)
   {
      List<Point3d> ret = new ArrayList<>();
      for (Point2d point2d : point2ds)
      {
         ret.add(new Point3d(point2d.x, point2d.y, z));
      }
      return ret;
   }

   private void filterOutPeaks()
   {
      long startTime = System.nanoTime();
      int nVerticesRemoved = 0;

      for (int i = 0; i < concaveHullVertices.size() - 2;)
      {
         Point2d a = concaveHullVertices.get(i);
         Point2d b = concaveHullVertices.get((i + 1) % concaveHullVertices.size());
         Point2d c = concaveHullVertices.get((i + 2) % concaveHullVertices.size());

         Vector2d ab = new Vector2d();
         Vector2d ac = new Vector2d();
         Vector2d bc = new Vector2d();
         ab.sub(b, a);
         ac.sub(c, a);
         bc.sub(c, b);
         ab.normalize();
         ac.normalize();
         bc.normalize();

         if (cross(ab, ac) < Math.sin(Math.toRadians(10.0)) && (Math.abs(ab.angle(bc)) > Math.toRadians(120.0) || Math.abs(ab.angle(bc)) < Math.toRadians(1.0)))
         {
            concaveHullVertices.remove(i + 1);
            nVerticesRemoved++;
         }
         else
            i++;
      }
      long endTime = System.nanoTime();
      System.out.println("filtering peaks took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime) + ", removed " + nVerticesRemoved + " vertices.");
   }

   private void filterOutShortEdges(double threshold)
   {
      long startTime = System.nanoTime();
      int nVerticesRemoved = 0;

      for (int i = 0; i < concaveHullVertices.size(); i++)
         nVerticesRemoved += throwShortEdgesRecursively(i, i + 1, threshold);
      long endTime = System.nanoTime();
      System.out.println("filtering short edges took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime) + ", removed " + nVerticesRemoved + " vertices.");
   }

   public int throwShortEdgesRecursively(int currentIndex, int nextIndex, double threshold)
   {
      int verticesRemoved = 0;
      Point2d a = concaveHullVertices.get(currentIndex);
      Point2d b = concaveHullVertices.get((nextIndex) % concaveHullVertices.size());
      Point2d c = concaveHullVertices.get((nextIndex + 1) % concaveHullVertices.size());

      Vector2d ab = new Vector2d();
      Vector2d ac = new Vector2d();
      ab.sub(b, a);
      ac.sub(c, a);
      double abLength = ab.length();
      ab.normalize();
      ac.normalize();

      if (abLength < threshold && cross(ab, ac) < Math.sin(Math.toRadians(10.0)))
      {
         verticesRemoved += throwShortEdgesRecursively(currentIndex, nextIndex + 1, threshold);
         if (verticesRemoved == 0)
         {
            concaveHullVertices.remove((nextIndex) % concaveHullVertices.size());
            verticesRemoved++;
         }
      }

      return verticesRemoved;
   }

   private void filterOutSmallTriangles()
   {
      long startTime = System.nanoTime();
      for (int i = 0; i < concaveHullVertices.size() - 2;)
      {
         Point2d a = concaveHullVertices.get(i);
         Point2d b = concaveHullVertices.get((i + 1) % concaveHullVertices.size());
         Point2d c = concaveHullVertices.get((i + 2) % concaveHullVertices.size());

         Vector2d ab = new Vector2d();
         Vector2d ac = new Vector2d();
         ab.sub(b, a);
         ac.sub(c, a);
         ab.normalize();
         ac.normalize();

         double area = Math.abs(a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y) / 2.0);
         if (area < 0.2 && cross(ab, ac) < Math.sin(Math.toRadians(1.0)))
            concaveHullVertices.remove(i + 1);
         else
            i++;
      }
      long endTime = System.nanoTime();
      System.out.println("filtering took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));
   }

   private void filter2(double threshold)
   {
      int nVerticesRemoved = 0;

      long startTime = System.nanoTime();
      Vector2d cutLine = new Vector2d();
      Vector2d currentToCandidate = new Vector2d();

      for (int i = 0; i < concaveHullVertices.size() - 1; i++)
      {
         Point2d currentVertex = concaveHullVertices.get(i);
         double index = -1;

         for (int j = i + concaveHullVertices.size() / 3; j >= i + 1 + 0*concaveHullVertices.size() / 6; j--)
         {
            Point2d other = concaveHullVertices.get(j % concaveHullVertices.size());

            if (currentVertex.distance(other) < threshold)
            {
               cutLine.sub(other, currentVertex);
               boolean areAllPointOutside = true;
               for (int k = i + 1; k < j; k++)
               {
                  Point2d candidate = concaveHullVertices.get(k % concaveHullVertices.size());
                  currentToCandidate.sub(candidate, currentVertex);
                  if (cross(cutLine, currentToCandidate) < 0.0)
                  {
                     areAllPointOutside = false;
                     break;
                  }
               }

               if (areAllPointOutside)
               {
                  index = j;
                  break;
               }
            }
         }

         if (index != -1)
         {
            for (int j = i + 1; j < index; j++)
            {
               concaveHullVertices.remove((i + 1) % concaveHullVertices.size());
               nVerticesRemoved++;
            }
         }
         else
            i++;

      }
      long endTime = System.nanoTime();
      System.out.println("filtering bumps took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime) + ", removed " + nVerticesRemoved + " vertices.");
   }

   private MultiPoint convertPoint2dToMultipoint(List<Point2d> points)
   {
      List<Coordinate> coordinates = new ArrayList<>();
      for (Point2d point : points)
      {
         coordinates.add(new Coordinate(point.getX(), point.getY()));
      }
      return new GeometryFactory().createMultiPoint(coordinates.toArray(new Coordinate[0]));
   }

   private List<Point2d> getGeometryVertices(Geometry geometry)
   {
      List<Point2d> vertices = new ArrayList<>();
      for (Coordinate vertex : geometry.getCoordinates())
         vertices.add(new Point2d(vertex.x, vertex.y));
      return vertices;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("OcTree Visualizer");

      Group rootNode = new Group();
      Scene scene = new Scene(rootNode, 600, 400, true);
      scene.setFill(Color.GRAY);
      rootNode.setMouseTransparent(true);
      setupCamera(rootNode, scene);
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      rootNode.getChildren().add(worldCoordinateSystem);

      primaryStage.setScene(scene);
      primaryStage.show();

      PointLight light = new PointLight(Color.WHITE);
      light.setTranslateZ(2.0);
      rootNode.getChildren().add(light);

      meshBuilder.clear();
      meshBuilder.addMultiLineMesh(toPoint3d(concaveHullVertices), 0.0025, Color.ALICEBLUE, true);
//      meshBuilder.addPolyon(getPoint3dsFromPolygon(convexPolygon2d, -0.001), Color.DARKBLUE);
      
      Random random = new Random(1561L);
      double z = 0.0;
      for (ConvexPolygon2d convexPolygon : decomposedPolygons)
      {
         double hue = 10.0 * 360.0 * random.nextDouble();
         meshBuilder.addPolyon(getPoint3dsFromPolygon(convexPolygon, z += 0.0), Color.hsb(hue, 1.0, 1.0));
      }
      
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      rootNode.getChildren().add(meshView);
      

      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         Sphere sphere = new Sphere(0.008);
         sphere.setMaterial(new PhongMaterial(Color.hsb(240.0 * i / concaveHullVertices.size(), 1.0, 1.0)));
         sphere.setTranslateX(concaveHullVertices.get(i).getX());
         sphere.setTranslateY(concaveHullVertices.get(i).getY());
         rootNode.getChildren().add(sphere);
      }

      for (Point2d vertex : pointCloud)
      {
         Box box = new Box(0.01, 0.01, 0.01);
         box.setMaterial(new PhongMaterial(Color.BLUEVIOLET));
         box.setTranslateX(vertex.getX());
         box.setTranslateY(vertex.getY());
         rootNode.getChildren().add(box);
      }

      for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
      {
         Sphere sphere = new Sphere(0.008);
         sphere.setMaterial(new PhongMaterial(Color.hsb(240.0 * i / convexPolygon2d.getNumberOfVertices(), 1.0, 1.0)));
         sphere.setTranslateX(convexPolygon2d.getVertex(i).getX());
         sphere.setTranslateY(convexPolygon2d.getVertex(i).getY());
         sphere.setTranslateZ(0.03);
         rootNode.getChildren().add(sphere);
      }
   }

   private List<Point3d> getPoint3dsFromPolygon(ConvexPolygon2d convexPolygon2d, double z)
   {
      List<Point3d> ret = new ArrayList<>();
      for (int i = convexPolygon2d.getNumberOfVertices() - 1; i >= 0; i--)
      {
         Point3d vertex3d = new Point3d();
         Point2d vertex2d = convexPolygon2d.getVertex(i);
         vertex3d.set(vertex2d.getX(), vertex2d.getY(), z);
         ret.add(vertex3d);
      }
      return ret;
   }

   private void setupCamera(Group root, Scene scene)
   {
      PerspectiveCamera camera = new PerspectiveCamera(true);
      camera.setNearClip(0.05);
      camera.setFarClip(50.0);
      scene.setCamera(camera);

      Vector3d up = new Vector3d(0.0, 0.0, 1.0);
      FocusBasedCameraMouseEventHandler cameraController = new FocusBasedCameraMouseEventHandler(scene.widthProperty(), scene.heightProperty(), camera, up);
      scene.addEventHandler(Event.ANY, cameraController);
      root.getChildren().add(cameraController.getFocusPointViz());
   }

   private static double cross(Vector2d v1, Vector2d v2)
   {
      return v1.getX() * v2.getY() - v1.getY() * v2.getX();
   }

   public static void main(String[] args)
   {
      Application.launch();
   }
}
