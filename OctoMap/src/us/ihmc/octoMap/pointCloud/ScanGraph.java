package us.ihmc.octoMap.pointCloud;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotics.geometry.RigidBodyTransform;

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
         System.err.println(getClass().getSimpleName() + " in addNode: scan is invalid.");
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
      if ((first.getId() != 0) && (second.getId() != 0))
      {
         ScanEdge ret = new ScanEdge(first, second, constraint);
         edges.add(ret);
         //      OCTOMAP_DEBUG("ScanGraph::AddEdge %d --> %d\n", first->id, second->id);
         return ret;
      }
      else
      {
         System.err.println(getClass().getSimpleName() + " in addEdge: one or both nodes invalid.");
         return null;
      }
   }

   public ScanEdge addEdge(int first_id, int second_id)
   {
      if (edgeExists(first_id, second_id))
      {
         System.err.println(getClass().getSimpleName() + " in addEdge: Edge already exists!");
         return null;
      }

      ScanNode first = getNodeByID(first_id);
      ScanNode second = getNodeByID(second_id);

      if ((first.getId() != 0) && (second.getId() != 0))
      {
         RigidBodyTransform constr = new RigidBodyTransform();
         constr.invert(first.getPose());
         constr.multiply(second.getPose());
         return addEdge(first, second, constr);
      }
      else
      {
         System.err.println(getClass().getSimpleName() + " in addEdge: one or both scans invalid.");
         return null;
      }
   }

   /// will return NULL if node was not found
   public ScanNode getNodeByID(int id)
   {
      for (int i = 0; i < nodes.size(); i++)
      {
         ScanNode currentNode = nodes.get(i);
         if (currentNode.getId() == id)
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
         ScanNode first = currentEdge.getFirst();
         ScanNode second = currentEdge.getSecond();
         if (((first.getId() == first_id) && (second.getId() == second_id)) || ((first.getId() == second_id) && (second.getId() == first_id)))
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
         c.invert(first.getPose());
         c.multiply(second.getPose());
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
            if (node.getId() == currentNode.getId())
               continue;
            if (edgeExists(id, currentNode.getId()))
            {
               res.add(currentNode.getId());
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
            if (currentScanEdge.getFirst().equals(node))
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
            if (currentScanEdge.getSecond().equals(node))
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
         scanNode.transformAbsoluteScan(scanNode.getPose());
      }
   }

   /// Cut graph (all containing Pointclouds) to given BBX in global coords
   public void crop(Point3d lowerBound, Point3d upperBound)
   {
      // for all node in graph...
      for (ScanNode scanNode : nodes)
      {
         RigidBodyTransform scan_pose = new RigidBodyTransform(scanNode.getPose());
         PointCloud pc = new PointCloud(scanNode.getScan());
         pc.transformAbsolute(scan_pose);
         pc.crop(lowerBound, upperBound);
         scan_pose.invert();
         pc.transform(scan_pose);
         scanNode.setScan(pc);;
      }
   }

   /// Cut Pointclouds to given BBX in local coords
   public void cropEachScan(Point3d lowerBound, Point3d upperBound)
   {
      for (ScanNode scanNode : nodes)
      {
         scanNode.cropScan(lowerBound, upperBound);
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
         retval += scanNode.getScanSize();
         if ((max_id > 0) && (scanNode.getId() == max_id))
            break;
      }
      return retval;
   }
}
