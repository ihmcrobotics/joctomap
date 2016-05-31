package us.ihmc.octoMap;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import javafx.application.Application;
import javafx.event.Event;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;

public class IntersectionPlaneBoxCalculatorVisualizer extends Application
{
   private final MeshBuilder meshBuilder = new MeshBuilder();
   private final IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();
   private final List<Point3d> intersections = new ArrayList<>();

   public IntersectionPlaneBoxCalculatorVisualizer()
   {
      Point3d boxCenter = new Point3d(0.0, 0.0, 0.3);
      calculator.setCube(1.0, boxCenter);
      Vector3d planeNormal = new Vector3d(0.0, 0.0, 1.0);
      Point3d planeOrigin = new Point3d();
      calculator.setPlane(planeOrigin, planeNormal);
      calculator.computeIntersections(intersections);

      Point3f v0 = new Point3f();
      Point3f v1 = new Point3f();
      Point3f v2 = new Point3f();
      Point2f tex = new Point2f();

      for (int i = 0; i < intersections.size(); i++)
      {
         v2.add(new Point3f(intersections.get(i)));
      }
      v2.scale(1.0f / intersections.size());

      for (int i = 0; i < intersections.size(); i++)
      {
         v0.set(intersections.get(i % intersections.size()));
         int index = (i + 1) % intersections.size();
         System.out.println(index);
         v1.set(intersections.get(index));
         meshBuilder.addSingleFace(v0, v1, v2, tex);
      }
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Group rootNode = new Group();
      Scene scene = new Scene(rootNode, 600, 400, true);
      scene.setFill(Color.GRAY);
      rootNode.setMouseTransparent(true);
      setupCamera(rootNode, scene);
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      rootNode.getChildren().add(worldCoordinateSystem);

      primaryStage.setScene(scene);
      primaryStage.show();

      MeshView meshView = new MeshView();
      meshView.setMesh(meshBuilder.generateMesh());
      PhongMaterial material = new PhongMaterial(Color.DARKCYAN);
      meshView.setMaterial(material);
      rootNode.getChildren().add(meshView);
      
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
      Application.launch(args);
   }
}
