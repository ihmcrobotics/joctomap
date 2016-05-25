package us.ihmc.octoMap.pointCloud;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;

public class ScanGraph
{

   protected List<ScanNode> nodes = new ArrayList<>();
   protected List<ScanEdge> edges = new ArrayList<>();

   /**
    * A ScanGraph is a collection of ScanNodes, connected by ScanEdges.
    * Each ScanNode contains a 3D scan performed from a pose.
    *
    */

   public ScanGraph()
   {
   }

   /// Clears all nodes and edges, and will delete the corresponding objects
   public void clear()
   {
      nodes.clear();
      edges.clear();
   }

   /**
    * Creates a new ScanNode in the graph from a Pointcloud.
    *
    * @param scan Pointer to a pointcloud to be added to the ScanGraph.
    *        ScanGraph will delete the object when it's no longer needed, don't delete it yourself.
    * @param pose 6D pose of the origin of the Pointcloud
    * @return Pointer to the new node
    */
   public ScanNode addNode(PointCloud scan, RigidBodyTransform pose)
   {
      if (scan.size() != 0)
      {
         ScanNode ret = new ScanNode(scan, pose, nodes.size());
         nodes.add(ret);
         return ret;
      }
      else
      {
         PrintTools.error(this, "scan is invalid.\n");
         return null;
      }
   }

   /**
    * Creates an edge between two ScanNodes.
    * ScanGraph will delete the object when it's no longer needed, don't delete it yourself.
    *
    * @param first ScanNode
    * @param second ScanNode
    * @param constraint 6D transform between the two nodes
    * @return
    */
   public ScanEdge addEdge(ScanNode first, ScanNode second, RigidBodyTransform constraint)
   {
      if ((first.id != 0) && (second.id != 0))
      {
         ScanEdge ret = new ScanEdge(first, second, constraint);
         edges.add(ret);
         //      OCTOMAP_DEBUG("ScanGraph::AddEdge %d --> %d\n", first->id, second->id);
         return ret;
      }
      else
      {
         PrintTools.error(this, "one or both nodes invalid.\n");
         return null;
      }
   }

   public ScanEdge addEdge(int first_id, int second_id)
   {
      if (edgeExists(first_id, second_id))
      {
         PrintTools.error(this, "Edge exists!\n");
         return null;
      }

      ScanNode first = getNodeByID(first_id);
      ScanNode second = getNodeByID(second_id);

      if ((first.id != 0) && (second.id != 0))
      {
         RigidBodyTransform constr = new RigidBodyTransform();
         constr.invert(first.pose);
         constr.multiply(second.pose);
         return addEdge(first, second, constr);
      }
      else
      {
         PrintTools.error(this, "one or both scans invalid.\n");
         return null;
      }
   }

   /// will return NULL if node was not found
   public ScanNode getNodeByID(int id)
   {
      for (int i = 0; i < nodes.size(); i++)
      {
         ScanNode currentNode = nodes.get(i);
         if (currentNode.id == id)
            return currentNode;
      }
      return null;
   }

   /// \return true when an edge between first_id and second_id exists
   public boolean edgeExists(int first_id, int second_id)
   {
      for (int i = 0; i < edges.size(); i++)
      {
         ScanEdge currentEdge = edges.get(i);
         ScanNode first = currentEdge.first;
         ScanNode second = currentEdge.second;
         if (((first.id == first_id) && (second.id == second_id)) || ((first.id == second_id) && (second.id == first_id)))
         {
            return true;
         }
      }
      return false;
   }

   /// Connect previously added ScanNode to the one before that
   public void connectPrevious()
   {
      if (nodes.size() >= 2)
      {
         ScanNode first = nodes.get(nodes.size() - 2);
         ScanNode second = nodes.get(nodes.size() - 1);
         RigidBodyTransform c = new RigidBodyTransform();
         c.invert(first.pose);
         c.multiply(second.pose);
         addEdge(first, second, c);
      }
   }

   public TIntArrayList getNeighborIDs(int id)
   {
      TIntArrayList res = new TIntArrayList();
      ScanNode node = getNodeByID(id);
      if (node != null)
      {
         // check all nodes
         for (int i = 0; i < nodes.size(); i++)
         {
            ScanNode currentNode = nodes.get(i);
            if (node.id == currentNode.id)
               continue;
            if (edgeExists(id, currentNode.id))
            {
               res.add(currentNode.id);
            }
         }
      }
      return res;
   }

   public List<ScanEdge> getOutEdges(ScanNode node)
   {
      List<ScanEdge> res = new ArrayList<>();
      if (node != null)
      {
         for (ScanEdge currentScanEdge : edges)
         {
            if (currentScanEdge.first.equals(node))
            {
               res.add(currentScanEdge);
            }
         }
      }
      return res;
   }

   // warning: constraints are reversed
   public List<ScanEdge> getInEdges(ScanNode node)
   {
      List<ScanEdge> res = new ArrayList<>();
      if (node != null)
      {
         for (ScanEdge currentScanEdge : edges)
         {
            if (currentScanEdge.second.equals(node))
            {
               res.add(currentScanEdge);
            }
         }
      }
      return res;
   }

   /// Transform every scan according to its pose
   public void transformScans()
   {
      for (ScanNode scanNode : nodes)
      {
         scanNode.scan.transformAbsolute(scanNode.pose);
      }
   }

   /// Cut graph (all containing Pointclouds) to given BBX in global coords
   public void crop(Point3d lowerBound, Point3d upperBound)
   {
      // for all node in graph...
      for (ScanNode scanNode : nodes)
      {
         RigidBodyTransform scan_pose = new RigidBodyTransform(scanNode.pose);
         PointCloud pc = new PointCloud(scanNode.scan);
         pc.transformAbsolute(scan_pose);
         pc.crop(lowerBound, upperBound);
         scan_pose.invert();
         pc.transform(scan_pose);
         scanNode.scan = pc;
      }
   }

   /// Cut Pointclouds to given BBX in local coords
   public void cropEachScan(Point3d lowerBound, Point3d upperBound)
   {
      for (ScanNode scanNode : nodes)
      {
         scanNode.scan.crop(lowerBound, upperBound);
      }
   }

   public int size()
   {
      return nodes.size();
   }

   public int getNumPoints()
   {
      return getNumPoints(-1);
   }

   public int getNumPoints(int max_id)
   {
      int retval = 0;

      for (ScanNode scanNode : nodes)
      {
         retval += scanNode.scan.size();
         if ((max_id > 0) && (scanNode.id == max_id))
            break;
      }
      return retval;
   }

   /**
    * A 3D scan as Pointcloud, performed from a Pose6D.
    */
   public static class ScanNode
   {
      PointCloud scan;
      RigidBodyTransform pose = new RigidBodyTransform(); ///< 6D pose from which the scan was performed
      private int id;

      public ScanNode(PointCloud scan, RigidBodyTransform pose, int id)
      {
         this.scan = new PointCloud(scan);
         this.pose.set(pose);
         this.id = id;
      }

      public ScanNode()
      {
         scan = null;
      }

      public ScanNode(ScanNode other)
      {
         this.scan = new PointCloud(other.scan);
         this.pose.set(other.pose);
         this.id = other.id;
      }

      public PointCloud getScan()
      {
         return scan;
      }

      public RigidBodyTransform getPose()
      {
         return pose;
      }

      public int getId()
      {
         return id;
      }

      public boolean equals(ScanNode other)
      {
         return id == other.id;
      }
   }

   /**
    * A connection between two \ref ScanNode "ScanNodes"
    */
   public static class ScanEdge
   {
      private ScanNode first;
      private ScanNode second;
      private RigidBodyTransform constraint = new RigidBodyTransform();
      private double weight;

      public ScanEdge(ScanNode first, ScanNode second, RigidBodyTransform constraint)
      {
         this.first = new ScanNode(first);
         this.second = new ScanNode(second);
         this.constraint.set(constraint);
         weight = 1.0;
      }

      public ScanEdge()
      {
         first = new ScanNode();
         second = new ScanNode();
      }

      public boolean equals(ScanEdge other)
      {
         return first.equals(other.first) && second.equals(other.second);
      }
   }
}
