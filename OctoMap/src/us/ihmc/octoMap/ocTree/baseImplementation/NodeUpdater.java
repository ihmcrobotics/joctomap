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

   public NODE updateNode(NODE root, OcTreeKeyReadOnly key)
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

      return updateNodeRecurs(root, createdRoot, key, 0);
   }

   private NODE updateNodeRecurs(NODE node, boolean nodeJustCreated, OcTreeKeyReadOnly key, int depth)
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

         NODE nodeChild = OcTreeNodeTools.getNodeChild(node, pos);

         if (updateRule.doLazyEvaluation())
         {
            return updateNodeRecurs(nodeChild, createdNode, key, depth + 1);
         }
         else
         {
            NODE leafToReturn = updateNodeRecurs(nodeChild, createdNode, key, depth + 1);
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

   private NODE createNodeChild(NODE node, int childIndex)
   {
      assignChildrenArrayIfNecessarry(node);

      NODE newChild = unusedNodes.isEmpty() ? nodeBuilder.createNode() : unusedNodes.remove(unusedNodes.size() - 1);
      node.setChild(childIndex, newChild);
      numberOfNodeCreatedDeleted++;

      return newChild;
   }

   private void assignChildrenArrayIfNecessarry(NODE node)
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
   private boolean pruneNode(NODE node)
   {
      if (!OcTreeNodeTools.isNodeCollapsible(node))
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

   private NODE getOrCreateNode()
   {
      return unusedNodes.isEmpty() ? nodeBuilder.createNode() : unusedNodes.remove(unusedNodes.size() - 1);
   }

   public interface UpdateRule<NODE extends AbstractOcTreeNode<NODE>>
   {
      public boolean doLazyEvaluation();

      public void updateLeaf(NODE leafToUpdate);

      public void updateInnerNode(NODE innerNodeToUpdate);
   }

   public interface EarlyAbortRule<NODE extends AbstractOcTreeNode<NODE>>
   {
      public boolean shouldAbortFullDepthUpdate(NODE nodeToUpdate);
   }
}
