package us.ihmc.octoMap;

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
import us.ihmc.octoMap.OcTreeIterator.LeafIterator;

public class OcTreeVisualizer extends Application
{
   public final OcTree ocTree = new OcTree(0.05);

   public OcTreeVisualizer()
   {
      ocTree.updateNode(0.0, 0.3, 0.0, true);
      ocTree.updateNode(0.3, 0.0, 0.0, true);
      ocTree.updateNode(0.0, 0.0, 0.3, true);
      
//      ocTree.coordToKeyChecked(x, y, z);
      
      
      System.out.println("Tree size: " + ocTree.size());
      
//      double dx = 0.001;
//      double dy = 0.001;
//      double dz = 0.001;
//
//      double xOff = 0.0;
//      double yOff = 0.0;
//      double zOff = 0.0;
//      // insert some measurements of occupied cells
//      for (int x = -20; x < 20; x++)
//      {
//         for (int y = -20; y < 20; y++)
//         {
//            for (int z = -20; z < 20; z++)
//            {
//               Point3d endpoint = new Point3d(x * dx + xOff, y * dy + yOff, z * dz + zOff);
//               ocTree.updateNode(endpoint, true);
//            }
//         }
//      }

      // set inner node colors
      ocTree.updateInnerOccupancy();
//      
//
//      // insert some measurements of free cells
//      for (int x = -30; x < 30; x++)
//      {
//         for (int y = -30; y < 30; y++)
//         {
//            for (int z = -30; z < 30; z++)
//            {
//               Point3d endpoint = new Point3d(x * 0.02f + 2.0f, y * 0.02f + 2.0f, z * 0.02f + 2.0f);
//               ocTree.updateNode(endpoint, false);
//            }
//         }
//      }

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

      LeafIterator<Float, OcTreeNode> it = ocTree.begin_leafs();
      LeafIterator<Float, OcTreeNode> end = ocTree.end_leafs();

      while (!it.equals(end))
      {
         if (ocTree.isNodeOccupied(it.getNode()))
         {
            double boxSize = it.getSize();
            Point3d boxCenter = it.getCoordinate();
            System.out.println("Size: " + boxSize + ", center: " + boxCenter);
            addBox(boxSize, boxCenter, rootNode);
         }
         
         it.next();
      }
   }

   private static final Color DEFAULT_COLOR = Color.DARKCYAN;
   
   public void addBox(double size, Point3d center, Group root)
   {
      Box box = new Box(size, size, size);
      box.setTranslateX(center.getX());
      box.setTranslateY(center.getY());
      box.setTranslateZ(center.getZ());
      PhongMaterial material = new PhongMaterial();
      material.setDiffuseColor(DEFAULT_COLOR);
      material.setSpecularColor(DEFAULT_COLOR.brighter());
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

      new OcTreeVisualizer();

      Application.launch(args);
   }
}
