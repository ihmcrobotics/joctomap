package us.ihmc.octoMap.ocTree.baseImplementation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.node.AbstractOcTreeNode;
import us.ihmc.octoMap.node.NodeBuilder;
import us.ihmc.octoMap.node.OcTreeNodeTools;
import us.ihmc.octoMap.tools.OcTreeKeyTools;
import us.ihmc.octoMap.tools.OcTreeSearchTools;

public class NodeUpdater<NODE extends AbstractOcTreeNode<NODE>>
{
   private NODE root;
   private int treeDepth;
   private int numberOfNodeCreatedDeleted;
   private UpdateRule<NODE> updateRule;
   private EarlyAbortRule<NODE> earlyAbortRule;
   private NodeBuilder<NODE> nodeBuilder;

   private final List<NODE> unusedNodes = new ArrayList<>();
   private final List<NODE[]> unusedNodeArrays = new ArrayList<>();

   public NodeUpdater(int treeDepth)
   {
      setTreeDepth(treeDepth);
   }

   public void setTreeDepth(int treeDepth)
   {
      this.treeDepth = treeDepth;
   }

   public int getNumberOfNodeCreatedDeleted()
   {
      return numberOfNodeCreatedDeleted;
   }

   public void setUpdateRule(UpdateRule<NODE> updateRule)
   {
      this.updateRule = updateRule;
   }

   public void setEarlyAbortRule(EarlyAbortRule<NODE> earlyAbortRule)
   {
      this.earlyAbortRule = earlyAbortRule;
   }

   public void setNodeBuilder(NodeBuilder<NODE> nodeBuilder)
   {
      this.nodeBuilder = nodeBuilder;
   }

   public NODE getRoot()
   {
      return root;
   }

   public NODE updateNode(NODE root, OcTreeKeyReadOnly key, boolean lazyEvaluation)
   {
      numberOfNodeCreatedDeleted = 0;

      boolean createdRoot = false;

      if (root == null)
      {
         root = getOrCreateNode();
         numberOfNodeCreatedDeleted++;
         createdRoot = true;
      }
      this.root = root;

      if (earlyAbortRule != null)
      {
         NODE leaf = OcTreeSearchTools.search(root, key, treeDepth);

         if (leaf != null)
         {
            if (earlyAbortRule.shouldAbortFullDepthUpdate(leaf))
               return leaf;
         }
      }

      return updateNodeRecurs(root, createdRoot, key, 0, lazyEvaluation);
   }

   protected NODE updateNodeRecurs(NODE node, boolean nodeJustCreated, OcTreeKeyReadOnly key, int depth, boolean lazyEvaluation)
   {
      boolean createdNode = false;

      if (node == null)
         throw new RuntimeException("The given node is null.");

      // follow down to last level
      if (depth < treeDepth)
      {
         int pos = OcTreeKeyTools.computeChildIndex(key, treeDepth - 1 - depth);
         if (!OcTreeNodeTools.nodeChildExists(node, pos))
         {
            // child does not exist, but maybe it's a pruned node?
            if (!node.hasAtLeastOneChild() && !nodeJustCreated)
            { // current node does not have children AND it is not a new node -> expand pruned node
               expandNode(node);
            }
            else
            { // not a pruned node, create requested child
               createNodeChild(node, pos);
               createdNode = true;
            }
         }

         if (lazyEvaluation)
         {
            return updateNodeRecurs(OcTreeNodeTools.getNodeChild(node, pos), createdNode, key, depth + 1, lazyEvaluation);
         }
         else
         {
            NODE leafToReturn = updateNodeRecurs(node.getChildUnsafe(pos), createdNode, key, depth + 1, lazyEvaluation);
            // prune node if possible, otherwise set own probability
            // note: combining both did not lead to a speedup!
            if (pruneNode(node)) // return pointer to current parent (pruned), the just updated node no longer exists
               leafToReturn = node;
            else // That's an inner node, apply the update rule
               updateRule.updateInnerNode(node);

            return leafToReturn;
         }
      }
      else // at last level, update node, end of recursion
      {
         updateRule.updateLeaf(node);
         return node;
      }
   }

   private void expandNode(NODE node)
   {
      assignChildrenArrayIfNecessarry(node);

      for (int i = 0; i < 8; i++)
      {
         NODE newChild = getOrCreateNode();
         newChild.copyData(node);
         node.setChild(i, newChild);
         numberOfNodeCreatedDeleted++;
      }
   }

   protected NODE createNodeChild(NODE node, int childIndex)
   {
      assignChildrenArrayIfNecessarry(node);

      NODE newChild = unusedNodes.isEmpty() ? node.create() : unusedNodes.remove(unusedNodes.size() - 1);
      node.setChild(childIndex, newChild);
      numberOfNodeCreatedDeleted++;

      return newChild;
   }

   public void assignChildrenArrayIfNecessarry(NODE node)
   {
      if (!node.hasArrayForChildren())
      {
         if (unusedNodeArrays.isEmpty())
            node.allocateChildren();
         else
            node.assignChildren(unusedNodeArrays.remove(unusedNodeArrays.size() - 1));
      }
   }

   /**
    * Prunes a node when it is collapsible
    * @return true if pruning was successful
    */
   public boolean pruneNode(NODE node)
   {
      if (!isNodeCollapsible(node))
         return false;

      // set value to children's values (all assumed equal)
      node.copyData(node.getChildUnsafe(0));

      // delete children (known to be leafs at this point!)
      for (int i = 0; i < 8; i++)
      {
         unusedNodes.add(node.removeChild(i));
         numberOfNodeCreatedDeleted--;
      }
      unusedNodeArrays.add(node.removeChildren());

      return true;
   }

   /**
    *  A node is collapsible if all children exist, don't have children of their own
    * and have the same occupancy value
    * @param node
    * @return
    */
   public boolean isNodeCollapsible(NODE node)
   {
      // All children must exist, must not have children of
      // their own and have the same occupancy probability
      if (!node.hasArrayForChildren())
         return false;

      NODE firstChild = node.getChild(0);
      if (firstChild == null || firstChild.hasAtLeastOneChild())
         return false;

      for (int i = 1; i < 8; i++)
      {
         NODE currentChild = node.getChild(i);

         if (currentChild == null || currentChild.hasAtLeastOneChild() || !currentChild.epsilonEquals(firstChild))
            return false;
      }
      return true;
   }

   private NODE getOrCreateNode()
   {
      return unusedNodes.isEmpty() ? nodeBuilder.createNode() : unusedNodes.remove(unusedNodes.size() - 1);
   }

   public interface UpdateRule<NODE extends AbstractOcTreeNode<NODE>>
   {
      public void updateLeaf(NODE leafToUpdate);

      public void updateInnerNode(NODE innerNodeToUpdate);
   }

   public interface EarlyAbortRule<NODE extends AbstractOcTreeNode<NODE>>
   {
      public boolean shouldAbortFullDepthUpdate(NODE nodeToUpdate);
   }
}
