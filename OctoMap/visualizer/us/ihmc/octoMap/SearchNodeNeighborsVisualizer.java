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
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.octoMap.tools.OcTreeKeyTools;

public class SearchNodeNeighborsVisualizer extends Application
{
   private static final Color NEIGHBOR_COLOR = new Color(Color.YELLOW.getRed(), Color.YELLOW.getGreen(), Color.YELLOW.getBlue(), 0.0);
   private final List<OcTreeKey> neighbors;
   private final int depth = 13;
   private final double resolution = 0.025;
   private final int treeDepth = 16;
   private final double searchRadius = 0.50;

   public SearchNodeNeighborsVisualizer()
   {
      double nodeSize = OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
      Point3d nodeCenter = new Point3d(nodeSize, nodeSize, nodeSize);
      nodeCenter.scale(0.5);

      OcTreeKey key = OcTreeKeyConversionTools.coordinateToKey(nodeCenter, depth, resolution, treeDepth);
      neighbors = OcTreeKeyTools.computeNeighborKeys(key, depth, resolution, treeDepth, searchRadius);
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("OcTree Visualizer");

      Group rootNode = new Group();
      Scene scene = new Scene(rootNode, 600, 400, true);
      scene.setFill(Color.GRAY);

      setupCamera(rootNode, scene);
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      rootNode.getChildren().add(worldCoordinateSystem);

      primaryStage.setScene(scene);
      primaryStage.show();
      
//      addFreeBox(0.2, new Point3d(0.1, 0.1, 0.1), rootNode);

      for (OcTreeKey ocTreeKey : neighbors)
      {
         double boxSize = OcTreeKeyConversionTools.computeNodeSize(depth, resolution, treeDepth);
         Point3d boxCenter = OcTreeKeyConversionTools.keyToCoordinate(ocTreeKey, depth, resolution, treeDepth);
//         System.out.println(boxCenter);
         addFreeBox(boxSize, boxCenter, rootNode);
      }
   }

   public void addFreeBox(double size, Point3d center, Group root)
   {
      addBox(size, center, root, NEIGHBOR_COLOR);
   }

   private void addBox(double size, Point3d center, Group root, Color color)
   {
      Box box = new Box(size, size, size);
      box.setTranslateX(center.getX());
      box.setTranslateY(center.getY());
      box.setTranslateZ(center.getZ());
      PhongMaterial material = new PhongMaterial();
      material.setDiffuseColor(color);
      box.setMaterial(material);
      root.getChildren().add(box);
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
