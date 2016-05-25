package us.ihmc.octoMap.ocTree;

import us.ihmc.octoMap.node.OcTreeDataNode;

public abstract class AbstractOcTree<NODE extends OcTreeDataNode<NODE>> extends OcTreeBaseImpl<NODE>
{
   public AbstractOcTree(double resolution)
   {
      super(resolution);
   }

   /// Constructor to enable derived classes to change tree constants.
   /// This usually requires a re-implementation of some core tree-traversal functions as well!
   protected AbstractOcTree(double resolution, int tree_depth, int tree_max_val)
   {
      super(resolution, tree_depth, tree_max_val);
   }

   public AbstractOcTree(AbstractOcTree<NODE> other)
   {
      super(other);
   }

   /// virtual constructor: creates a new object of same type
   public abstract AbstractOcTree<NODE> create();

   /// returns actual class name as string for identification
   public abstract String getTreeType();
}
