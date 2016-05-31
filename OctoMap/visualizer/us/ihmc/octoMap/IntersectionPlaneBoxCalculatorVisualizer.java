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
import javafx.scene.shape.Box;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;
import us.ihmc.octoMap.tools.IntersectionPlaneBoxCalculator;

public class IntersectionPlaneBoxCalculatorVisualizer extends Application
{
   private final MeshBuilder meshBuilder = new MeshBuilder();
   private final IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();
   private final List<Point3d> intersections = new ArrayList<>();
   private final Box box;

   public IntersectionPlaneBoxCalculatorVisualizer()
   {
      double lx = 1.0;
      double ly = 1.0;
      double lz = 1.0;

      Point3d boxCenter = new Point3d(0.0, 0.0, 0.3);
      calculator.setBox(lx, ly, lz, boxCenter);
      Vector3d planeNormal = new Vector3d(0.0, 1.0, 1.0);
      planeNormal.normalize();
      Point3d planeOrigin = new Point3d();
      calculator.setPlane(planeOrigin, planeNormal);
      calculator.computeIntersections(intersections);
      meshBuilder.addPolygon(intersections);

      box = new Box(lx, ly, lz);
      box.setTranslateX(boxCenter.x);
      box.setTranslateY(boxCenter.y);
      box.setTranslateZ(boxCenter.z);
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

      material = new PhongMaterial();
      material.setDiffuseColor(new Color(1.0, 1.0, 0.0, 0.0));
      box.setMaterial(material);
      rootNode.getChildren().add(box);
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
