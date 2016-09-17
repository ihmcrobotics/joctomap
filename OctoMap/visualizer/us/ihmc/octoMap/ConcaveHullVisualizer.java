package us.ihmc.octoMap;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point3d;
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
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.robotics.time.TimeTools;

public class ConcaveHullVisualizer extends Application
{
   private final List<Point3d> pointCloud = new ArrayList<>();
   private final List<Point3d> concaveHullVertices = new ArrayList<>();
   private final MultiColorMeshBuilder meshBuilder = new MultiColorMeshBuilder();

   public ConcaveHullVisualizer() throws IOException
   {
      Random random = new Random(34254);
      List<Coordinate> coordinates = new ArrayList<>();
      GeometryFactory geometryFactory = new GeometryFactory();

      InputStreamReader inputStreamReader = new InputStreamReader(getClass().getResourceAsStream("regionPoints"));
      BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

      String line = "";
      String cvsSplitBy = ",";

      while ((line = bufferedReader.readLine()) != null)
      {
         String[] coordsAsString = line.split(cvsSplitBy);
         Point3d point = new Point3d();
         double x = Double.parseDouble(coordsAsString[0]);
         point.setX(x);
         double y = Double.parseDouble(coordsAsString[1]);
         point.setY(y);
//         point.setZ(Double.parseDouble(coordsAsString[2]));
         coordinates.add(new Coordinate(x, y));
         pointCloud.add(point);
      }
      
//      for (int i = 0; i < 100; i++)
//      {
//         double x = random.nextDouble();
//         double y = random.nextDouble();
//         Coordinate coordinate = new Coordinate(x, y);
//         coordinates.add(coordinate);
//         pointCloud.add(new Point3d(x, y, 0));
//      }

      MultiPoint multiPoint = geometryFactory.createMultiPoint(coordinates.toArray(new Coordinate[0]));
      
      long startTime = System.nanoTime();
      
      ConcaveHull concaveHull = new ConcaveHull(multiPoint, 0.1);
      Geometry concaveHullGeometry = concaveHull.getConcaveHull();
      
      for (Coordinate vertex : concaveHullGeometry.getCoordinates())
      {
         
         concaveHullVertices.add(new Point3d(vertex.x, vertex.y, 0.0));
      }
      
      long endTime = System.nanoTime();
      System.out.println("Took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

      System.out.println(concaveHullVertices.size());
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

      meshBuilder.clear();
      meshBuilder.addMultiLineMesh(concaveHullVertices, 0.005, Color.ALICEBLUE, true);
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      rootNode.getChildren().add(meshView);
      
      for (Point3d vertex : pointCloud)
      {
         Box box = new Box(0.01, 0.01, 0.01);
         box.setMaterial(new PhongMaterial(Color.BLUEVIOLET));
         box.setTranslateX(vertex.getX());
         box.setTranslateY(vertex.getY());
         rootNode.getChildren().add(box);
      }
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

   public static void main(String[] args)
   {
      Application.launch();
   }
}
