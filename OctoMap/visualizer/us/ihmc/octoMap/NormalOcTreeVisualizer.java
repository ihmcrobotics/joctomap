package us.ihmc.octoMap;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import javax.vecmath.Matrix3d;
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
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.octoMap.iterators.LeafIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.NormalOcTree;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.time.TimeTools;

public class NormalOcTreeVisualizer extends Application
{
   public final NormalOcTree ocTree = new NormalOcTree(0.15);
   private static final boolean SHOW_FREE_CELLS = false;
   private static final Color FREE_COLOR = new Color(Color.YELLOW.getRed(), Color.YELLOW.getGreen(), Color.YELLOW.getBlue(), 0.0);

   public NormalOcTreeVisualizer()
   {

      //      callUpdateNode();
      //      callInsertPointCloud();
//      createPlane(0.0, 0.0, -0.05);
      createBowl(5.0, new Point3d());
      System.out.println("Number of leafs: " + ocTree.getNumLeafNodes());
      System.out.println("Initialized octree");
      System.out.println("Computing normals");
      long startTime = System.nanoTime();
      ocTree.updateNormals();
      long endTime = System.nanoTime();
      System.out.println("Done computing normals: time it took = " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

      Point3d badNodeCoordinate = new Point3d(0.975, -0.025, -0.075);
      OcTreeKey badNodeKey = ocTree.coordinateToKey(badNodeCoordinate);
      
      HashSet<OcTreeKey> neighborKeySet = new HashSet<>();

      // There is 8 neighbouring sets
      // The current cube can be at any of the 8 vertex
      int[][] xIndex = new int[][] {{1, 1, 0, 0}, {1, 1, 0, 0}, {0, 0, - 1, -1}, {0, 0, - 1, -1}};
      int[][] yIndex = new int[][] {{1, 0, 0, 1}, {0, -1, -1, 0}, {0, -1, -1, 0}, {1, 0, 0, 1}};
      int[][] zIndex = new int[][] {{0, 1}, {-1, 0}};

      // Iterate over the 8 neighboring sets
      for (int m = 0; m < 2; ++m)
      {
         for (int l = 0; l < 4; ++l)
         {
            int k = 0;
            // Iterate over the cubes
            for (int j = 0; j < 2; ++j)
            {
               for (int i = 0; i < 4; ++i)
               {
                  OcTreeKey currentKey = new OcTreeKey();
                  currentKey.setKey(0, badNodeKey.getKey(0) + xIndex[l][i]);
                  currentKey.setKey(1, badNodeKey.getKey(1) + yIndex[l][i]);
                  currentKey.setKey(2, badNodeKey.getKey(2) + zIndex[m][j]);

                  neighborKeySet.add(currentKey);
               }
            }
         }
      }

      NumberFormat format = new DecimalFormat(" 0.000;-0.000");
      for (OcTreeKey key : neighborKeySet)
      {
         Point3d coord = ocTree.keyToCoordinate(key);
         NormalOcTreeNode node = ocTree.search(key);
         System.out.println("(" + format.format(coord.getX()) + ", " + format.format(coord.getY()) + ", " + format.format(coord.getZ()) +"), node: " + node);
      }

      
      System.out.println("in getNormals()");
      List<Vector3d> normals = new ArrayList<>();
      ocTree.getNormals(badNodeKey, normals, true);
      System.out.println(normals);

      Point3d end = new Point3d();
      Vector3d direction = new Vector3d(-3.54, -3.565, -0.2);
      direction.normalize();
      ocTree.castRay(new Point3d(), direction, end);
      
      NormalOcTreeNode search = ocTree.search(end);
      if (search != null)
         System.out.println(search.getPlane());
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

      for (double x = -0.5 * planeSize; x < 0.5 * planeSize; x += 0.7 * ocTree.getResolution())
      {
         for (double y = -0.5 * planeSize; y < 0.5 * planeSize; y += 0.7 * ocTree.getResolution())
         {
            Point3d point = new Point3d(x, y, 0.0);
            Matrix3d rotation = new Matrix3d();
            RotationTools.convertYawPitchRollToMatrix(0.0, Math.toRadians(pitch), Math.toRadians(roll), rotation);
            rotation.transform(point);
            point.setZ(point.getZ() + z);

            pointcloud.add(point);
         }
      }
      ocTree.insertPointCloud(pointcloud, origin);
   }

   public void createBowl(double radius, Point3d center)
   {
      Point3d origin = new Point3d(0.0, 0.0, center.getZ() + 0.0);


      double res = 0.02;
      for (double yaw = 0.0; yaw < 2.0 * Math.PI; yaw += res)
      {
         for (double pitch = 0.0; pitch < 0.5 * Math.PI; pitch += res)
         {
            double x = Math.cos(pitch) * Math.cos(yaw) * radius + center.getX();
            double y = Math.cos(pitch) * Math.sin(yaw) * radius + center.getY();
            double z = - Math.sin(pitch) * radius + center.getZ();
            pointcloud.add(x, y, z);
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
      rootNode.setMouseTransparent(true);
      setupCamera(rootNode, scene);
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      rootNode.getChildren().add(worldCoordinateSystem);

      primaryStage.setScene(scene);
      primaryStage.show();

      TextureColorPalette1D palette = new TextureColorPalette1D();
      palette.setHueBased(0.9, 0.8);
      MultiColorMeshBuilder occupiedMeshBuilder = new MultiColorMeshBuilder(palette);
      MeshBuilder freeMeshBuilder = new MeshBuilder();

      LeafIterable<NormalOcTreeNode> leafIterable = new LeafIterable<>(ocTree);
      for (OcTreeSuperNode<NormalOcTreeNode> node : leafIterable)
      {
         double boxSize = node.getSize();
         Point3d boxCenter = node.getCoordinate();

         if (ocTree.isNodeOccupied(node.getNode()))
         {
            Vector3d normal = node.getNode().getNormal();
            Color normalBasedColor = getNormalBasedColor(normal);
            List<Point3d> plane = node.getNode().getPlane();
            if (plane != null)
               occupiedMeshBuilder.addPolyon(plane, normalBasedColor);
            else
               occupiedMeshBuilder.addCubeMesh((float) boxSize, new Point3f(boxCenter), normalBasedColor);
         }
         else if (SHOW_FREE_CELLS)
         {
            freeMeshBuilder.addCubeMesh((float) boxSize, new Point3f(boxCenter));
         }
      }

      MeshView occupiedMeshView = new MeshView();
      occupiedMeshView.setMesh(occupiedMeshBuilder.generateMesh());
      occupiedMeshView.setMaterial(occupiedMeshBuilder.generateMaterial());
      rootNode.getChildren().add(occupiedMeshView);

      if (SHOW_FREE_CELLS)
      {
         MeshView freeMeshView = new MeshView();
         freeMeshView.setMesh(freeMeshBuilder.generateMesh());
         PhongMaterial material = new PhongMaterial();
         material.setDiffuseColor(FREE_COLOR);
         freeMeshView.setMaterial(material);
         rootNode.getChildren().add(freeMeshView);
      }
   }

   private static final Color DEFAULT_COLOR = Color.DARKCYAN;

   public Color getNormalBasedColor(Vector3d normal)
   {
      Color color = DEFAULT_COLOR;

      if (normal != null)
      {
         Vector3d zUp = new Vector3d(0.0, 0.0, 1.0);
         normal.normalize();
         double angle = Math.abs(zUp.dot(normal));
         double hue = 120.0 * angle;
         color = Color.hsb(hue, 1.0, 1.0);
      }
      return color;
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
