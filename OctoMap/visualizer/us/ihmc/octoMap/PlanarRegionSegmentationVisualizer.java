package us.ihmc.octoMap;

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
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.octoMap.iterators.LeafIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.planarRegions.PlanarRegion;
import us.ihmc.octoMap.pointCloud.PointCloud;
import us.ihmc.octoMap.tools.IntersectionPlaneBoxCalculator;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class PlanarRegionSegmentationVisualizer extends Application
{
   public final NormalOcTree ocTree = new NormalOcTree(0.01);
   private static final double TWO_PI = 2.0 * Math.PI;
   private static final boolean SHOW_POINT_CLOUD = false;

   public PlanarRegionSegmentationVisualizer()
   {
      Point3d lidarLocation = new Point3d(0.0, 0.0, 5.0);
//      createPlane(-20.0, 0.0, new Vector3d(0.1, 0.0, 0.0), lidarLocation);
      createSawToothPlanes(0.0, 0.0, new Vector3d(), lidarLocation);
      ocTree.updateHitLocations(lidarLocation, pointcloud, 0.05);
      ocTree.updateNormalsAndPlanarRegions(16);
      ocTree.updateNormalsAndPlanarRegions(16);
      ocTree.updateNormalsAndPlanarRegions(16);
      ocTree.updateNormalsAndPlanarRegions(16);
      ocTree.updateNormalsAndPlanarRegions(16);
   }
   
   PointCloud pointcloud = new PointCloud();

   public void createPlane(double pitch, double roll, Vector3d planeOffset, Point3d lidarLocation)
   {
      Point3d origin = new Point3d(lidarLocation);
      pointcloud.clear();

      double planeSize = 0.5;

      double d = 0.2 * ocTree.getResolution();
      for (double x = -0.5 * planeSize; x < 0.5 * planeSize; x += d)
      {
         for (double y = -0.5 * planeSize; y < 0.5 * planeSize; y += d)
         {
            Point3d point = new Point3d(x, y, 0.0);
            Matrix3d rotation = new Matrix3d();
            RotationTools.convertYawPitchRollToMatrix(0.0, Math.toRadians(pitch), Math.toRadians(roll), rotation);
            rotation.transform(point);
            point.add(planeOffset);

            pointcloud.add(point);
         }
      }
      ocTree.insertPointCloud(pointcloud, origin);
   }

   public void createSawToothPlanes(double pitch, double roll, Vector3d planeOffset, Point3d lidarLocation)
   {
      Point3d origin = new Point3d(lidarLocation);
      pointcloud.clear();

      double toothAmplitude = 0.12;
      double toothFrequency = 0.6;
      double length = 10.0;
      double planeWidth = 0.5;

      for (double x = -0.5 * length; x < 0.5 * length; x += 0.7 * ocTree.getResolution())
      {
         for (double y = -0.5 * planeWidth; y < 0.5 * planeWidth; y += 0.7 * ocTree.getResolution())
         {
            Point3d point = new Point3d(x, y, 0.0);

            double angle = (TWO_PI * toothFrequency * x) % TWO_PI;
            if (angle < 0.0)
               angle = angle + TWO_PI;

            if (angle < Math.PI / 2.0)
            {
               double percentUp = angle / (Math.PI / 2.0);
               point.setZ(toothAmplitude * percentUp);
            }
            else if (angle < 3.0 * Math.PI / 2.0)
            {
               double percentDown = (angle - Math.PI / 2.0) / (Math.PI / 2.0);
               point.setZ(toothAmplitude * (1.0 - percentDown));
            }
            else if (angle < TWO_PI)
            {
               double percentUp = (angle - 3.0 * Math.PI / 2.0) / (Math.PI / 2.0);
               point.setZ(toothAmplitude * (percentUp - 1.0));
            }
            else
            {
               point.setZ(0.0);
            }
            
            Matrix3d rotation = new Matrix3d();
            RotationTools.convertYawPitchRollToMatrix(0.0, Math.toRadians(pitch), Math.toRadians(roll), rotation);
            rotation.transform(point);
            point.add(planeOffset);

            pointcloud.add(point);
         }
      }
      ocTree.insertPointCloud(pointcloud, origin);
   }

   private final RecyclingArrayList<Point3d> plane = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3d.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

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

      Vector3d nodeNormal = new Vector3d();
      Point3d pointOnPlane = new Point3d();

      LeafIterable<NormalOcTreeNode> leafIterable = new LeafIterable<>(ocTree, 16);
      for (OcTreeSuperNode<NormalOcTreeNode> superNode : leafIterable)
      {
         double boxSize = superNode.getSize();
         Point3d boxCenter = superNode.getCoordinate();

         NormalOcTreeNode node = superNode.getNode();
         if (ocTree.isNodeOccupied(node))
         {
            int regionId = node.getRegionId();
            Color normalBasedColor = getPlanarRegionBasedColor(regionId);
            if (node.isNormalSet())
            {
               node.getNormal(nodeNormal);
               if (node.isCenterSet())
               {
                  node.getCenter(pointOnPlane);
//                  occupiedMeshBuilder.addCubeMesh(0.005, pointOnPlane, normalBasedColor);
               }
               else
                  pointOnPlane.set(boxCenter);
               intersectionPlaneBoxCalculator.setCube(boxSize, boxCenter);
               intersectionPlaneBoxCalculator.setPlane(pointOnPlane, nodeNormal);
               intersectionPlaneBoxCalculator.computeIntersections(plane);
               occupiedMeshBuilder.addPolyon(plane, normalBasedColor);
            }
            else
            {

               if (node.isCenterSet())
               {
                  node.getCenter(pointOnPlane);
                  occupiedMeshBuilder.addCubeMesh(0.005, pointOnPlane, normalBasedColor);
               }
               else
               {
                  occupiedMeshBuilder.addCubeMesh((float) boxSize, new Point3f(boxCenter), normalBasedColor);
               }
            }
         }
      }

      MeshView occupiedMeshView = new MeshView();
      occupiedMeshView.setMesh(occupiedMeshBuilder.generateMesh());
      occupiedMeshView.setMaterial(occupiedMeshBuilder.generateMaterial());
      rootNode.getChildren().add(occupiedMeshView);

      if (SHOW_POINT_CLOUD)
      {
         for (int i = 0; i < pointcloud.size(); i++)
         {
            Sphere sphere = new Sphere(0.0025);
            sphere.setTranslateX(pointcloud.getPoint(i).getX());
            sphere.setTranslateY(pointcloud.getPoint(i).getY());
            sphere.setTranslateZ(pointcloud.getPoint(i).getZ());
            rootNode.getChildren().add(sphere);
         }
      }
   }

   private static final Color DEFAULT_COLOR = Color.DARKCYAN;

   public Color getPlanarRegionBasedColor(int regionId)
   {
      Color color = DEFAULT_COLOR;

      if (regionId != PlanarRegion.NO_REGION_ID)
      {
         java.awt.Color awtColor = new java.awt.Color(regionId);
         color = Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
      }
      return color;
   }

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
