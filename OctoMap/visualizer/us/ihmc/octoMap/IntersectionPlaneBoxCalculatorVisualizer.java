package us.ihmc.octoMap;

import java.util.List;

import javax.vecmath.Point3d;
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
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.octoMap.tools.IntersectionPlaneBoxCalculator;

public class IntersectionPlaneBoxCalculatorVisualizer extends Application
{
   private final TextureColorPalette1D colorPalette = new TextureColorPalette1D();
   private final MultiColorMeshBuilder colorMeshBuilder = new MultiColorMeshBuilder(colorPalette);
   private final IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();
   private final Box box;
//   private final Cylinder normalCylinder;

   public IntersectionPlaneBoxCalculatorVisualizer()
   {
      double lx = 1.0;
      double ly = 1.0;
      double lz = 1.0;

      Point3d cubeCenter = new Point3d(-4.5311850804994585, -4.933378120148582, 6.517879816335377);
      Point3d pointOnPlane = new Point3d(-4.216341440000136, -4.90149655983476, 6.713644934521557);
      Vector3d planeNormal = new Vector3d(-0.750449710466656, 0.40248521745139515, -0.5242431514036355);

//      pointOnPlane.sub(cubeCenter);
//      cubeCenter.set(0.0, 0.0, 0.0);

      calculator.setBox(lx, ly, lz, cubeCenter);
      calculator.setPlane(pointOnPlane, planeNormal);

      colorPalette.setHueBased(0.9, 0.8);

      List<Point3d> intersections = calculator.computeIntersections();
      System.out.println(intersections);
      
      colorMeshBuilder.addPolyon(intersections, Color.DARKCYAN);
      for (int index = 0; index < intersections.size(); index++)
         colorMeshBuilder.addCubeMesh(0.01, intersections.get(index), Color.FIREBRICK);
      colorMeshBuilder.addCubeMesh(0.02, pointOnPlane, Color.SLATEGREY);
      box = new Box(lx, ly, lz);
      box.setTranslateX(cubeCenter.getX());
      box.setTranslateY(cubeCenter.getY());
      box.setTranslateZ(cubeCenter.getZ());

//      normalCylinder = new Cylinder(0.01, 0.3);
//      normalCylinder.setTranslateX(pointOnPlane.getX());
//      normalCylinder.setTranslateY(pointOnPlane.getY() + 0.5 * normalCylinder.getHeight());
//      normalCylinder.setTranslateZ(pointOnPlane.getZ());
//      AxisAngle4d axisAngle = new AxisAngle4d();
//      GeometryTools.getRotationBasedOnNormal(axisAngle, planeNormal, new Vector3d(0.0, 1.0, 0.0));
//      Affine affine = new Affine();
//      JavaFXTools.convertAxisAngleToAffine(axisAngle, affine);
//      normalCylinder.getTransforms().add(affine);
//      
      for (int i = 0; i < intersections.size(); i++)
      {

         Point3d intersection = intersections.get(i);
         Vector3d v0 = new Vector3d();
         Vector3d v1 = new Vector3d();
         Vector3d v3 = new Vector3d();
         Point3d nextIntersection = intersections.get((i + 1) % intersections.size());
         Point3d previousIntersection = intersections.get(i == 0 ? intersections.size() - 1 : i - 1);
         v0.sub(intersection, nextIntersection);
         v1.sub(intersection, previousIntersection);
         v3.cross(v0, v1);
         System.out.println(v3.dot(planeNormal) < 0.0);
      }
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Group rootNode = new Group();
      Scene scene = new Scene(rootNode, 800, 600, true);
      scene.setFill(Color.GRAY);
      rootNode.setMouseTransparent(true);
      setupCamera(rootNode, scene);
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      rootNode.getChildren().add(worldCoordinateSystem);

      primaryStage.setScene(scene);
      primaryStage.show();

      MeshView meshView = new MeshView();
      meshView.setMesh(colorMeshBuilder.generateMesh());
      meshView.setMaterial(colorMeshBuilder.generateMaterial());
      rootNode.getChildren().add(meshView);

      PhongMaterial material = new PhongMaterial(Color.DARKCYAN);
//      normalCylinder.setMaterial(material);
//      rootNode.getChildren().add(normalCylinder);
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
