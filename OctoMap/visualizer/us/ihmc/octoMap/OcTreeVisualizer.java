package us.ihmc.octoMap;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import javax.vecmath.Matrix3d;
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
import us.ihmc.octoMap.iterators.LeafIterable;
import us.ihmc.octoMap.iterators.OcTreeIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.node.OccupancyOcTreeNode;
import us.ihmc.octoMap.ocTree.OcTree;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.robotics.geometry.RotationTools;

public class OcTreeVisualizer extends Application
{
   public final OcTree ocTree = new OcTree(0.05);
   private static final boolean SHOW_FREE_CELLS = false;
   private static final Color FREE_COLOR = new Color(Color.YELLOW.getRed(), Color.YELLOW.getGreen(), Color.YELLOW.getBlue(), 0.0);

   public OcTreeVisualizer()
   {

//      callUpdateNode();
//      callInsertPointCloud();
      createPlane(45, 0.0, -0.05);
      
//      pointcloud.add(new Point3d());
//      ocTree.insertPointCloud(pointcloud, new Point3d(0.0, 0.0, 2.0));
      
      int numberOfNodes = 0;
      int numberOfLeafs = 0;
      OccupancyOcTreeNode root = ocTree.getRoot();
      List<OccupancyOcTreeNode> stack = new ArrayList<>();

      if (root != null)
      {
         numberOfNodes++;
         stack.add(root);
         
         while (!stack.isEmpty())
         {
            OccupancyOcTreeNode currentNode = stack.remove(0);
            if (currentNode.hasAtLeastOneChild())
            {
               for (int i = 0; i < 8; i++)
               {
                  OccupancyOcTreeNode currentChild = (OccupancyOcTreeNode) currentNode.getChildUnsafe(i);
                  if (currentChild != null)
                  {
                     numberOfNodes++;
                     stack.add(currentChild);
                  }
               }
            }
            else
            {
               numberOfLeafs++;
            }
         }
      }
      System.out.println("Computed number of nodes = " + numberOfNodes);
      System.out.println("Computed number of leafs = " + numberOfLeafs);
      
      HashSet<OccupancyOcTreeNode> foundNodes = new HashSet<>();
      int iteratorDuplicatedFounds = 0;
      int iteratorNodeCount = 0;
      int iteratorLeafCount = 0;
      
      OcTreeIterable<OccupancyOcTreeNode> treeIterable = new OcTreeIterable<>(ocTree);
      for (OcTreeSuperNode<OccupancyOcTreeNode> node : treeIterable)
      {
         iteratorNodeCount++;
         if (foundNodes.contains(node))
            iteratorDuplicatedFounds++;
         foundNodes.add(node.getNode());
         if (node.isLeaf())
            iteratorLeafCount++;
      }
      
      System.out.println("Iterator node count = " + iteratorNodeCount);
      System.out.println("Iterator leaf count = " + iteratorLeafCount);
      System.out.println("Iterator duplicated count = " + iteratorDuplicatedFounds);

      System.out.println("Tree size: " + ocTree.size());


//      long startTime = System.nanoTime();
//      for (int i = 0; i < 10000; i++)
//      {
         Point3d end = new Point3d(5.0, 3.0, 1.0);
         ocTree.castRay(new Point3d(0.0, 0.0, 2.0), new Vector3d(0.0, 0.0, -1.0), end, true);
         List<Vector3d> normals = new ArrayList<>();
         ocTree.getNormals(end, normals);
//      }
      System.out.println(normals);
//      long endTime = System.nanoTime();
//      System.out.println(TimeTools.nanoSecondstoSeconds(endTime - startTime));
   }

   PointCloud pointcloud = new PointCloud();

   private void callInsertPointCloud()
   {
      Point3d origin = new Point3d(0.01, 0.01, 0.02);
      Point3d pointOnSurface = new Point3d(4.01, 0.01, 0.01);


      for (int i = 0; i < 100; i++)
      {
         for (int j = 0; j < 100; j++)
         {
            Point3d rotated = new Point3d(pointOnSurface);
            Matrix3d rotation = new Matrix3d();
            RotationTools.convertYawPitchRollToMatrix(Math.toRadians(i * 0.5), Math.toRadians(j * 0.5), 0.0, rotation);
            rotation.transform(rotated);
            pointcloud.add(rotated);
         }
      }

      ocTree.insertPointCloud(pointcloud, origin);
   }

   private void callUpdateNode()
   {
      double dx = 0.05;
      double dy = 0.05;
      double dz = 0.05;

      double xOff = 0.01;
      double yOff = 0.01;
      double zOff = 0.01;
      // insert some measurements of occupied cells
      for (int x = -20; x < 20; x++)
      {
         for (int y = -20; y < 20; y++)
         {
            for (int z = -20; z < 20; z++)
            {
               Point3d endpoint = new Point3d(x * dx + xOff, y * dy + yOff, z * dz + zOff);
               ocTree.updateNode(endpoint, true);
            }
         }
      }

//      // insert some measurements of free cells
      for (int x = -30; x < 30; x++)
      {
         for (int y = -30; y < 30; y++)
         {
            for (int z = -30; z < 30; z++)
            {
               Point3d endpoint = new Point3d(x * 0.02f + 2.0f, y * 0.02f + 2.0f, z * 0.02f + 2.0f);
               ocTree.updateNode(endpoint, false);
            }
         }
      }

      ocTree.updateInnerOccupancy();
   }

   public void createPlane(double pitch, double roll, double z)
   {
      Point3d origin = new Point3d(0.0, 0.0, z + 2.0);
      pointcloud.clear();

      double planeSize = 4.0;

      for (double x = -0.5*planeSize; x < 0.5*planeSize; x+= 0.7 * ocTree.getResolution())
      {
         for (double y = -0.5*planeSize; y < 0.5*planeSize; y+= 0.7 * ocTree.getResolution())
         {
            Point3d point = new Point3d(x, y, 0.0);
            Matrix3d rotation = new Matrix3d();
            RotationTools.convertYawPitchRollToMatrix(0.0, Math.toRadians(pitch), Math.toRadians(roll), rotation);
            rotation.transform(point);
            point.z += z;

            pointcloud.add(point);
         }
      }
      ocTree.insertPointCloud(pointcloud, origin);
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

      LeafIterable<OccupancyOcTreeNode> leafIterable = new LeafIterable<>(ocTree);
      for (OcTreeSuperNode<OccupancyOcTreeNode> node : leafIterable)
      {
         double boxSize = node.getSize();
         Point3d boxCenter = node.getCoordinate();

         if (ocTree.isNodeOccupied(node.getNode()))
         {
            addOccupiedBox(boxSize, boxCenter, rootNode);
         }
         else if (SHOW_FREE_CELLS)
         {
            addFreeBox(boxSize, boxCenter, rootNode);
         }
      }
   }

   private static final Color DEFAULT_COLOR = Color.DARKCYAN;

   public void addOccupiedBox(double size, Point3d center, Group root)
   {
      addBox(size, center, root, DEFAULT_COLOR);
   }

   public void addFreeBox(double size, Point3d center, Group root)
   {
      addBox(size, center, root, FREE_COLOR);
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

//      new OcTreeVisualizer();

      Application.launch(args);
   }
}
