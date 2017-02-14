package us.ihmc.jOctoMap.testTools;

import java.util.Random;

import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.jOctoMap.rules.interfaces.UpdateRule;

public class TestOcTree extends AbstractOcTreeBase<TestOcTreeNode>
{
   private final UpdateRule<TestOcTreeNode> doNothingRule = new UpdateRule<TestOcTreeNode>()
   {
      @Override
      public void updateLeaf(TestOcTreeNode leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated)
      {
      }

      @Override
      public void updateInnerNode(TestOcTreeNode innerNodeToUpdate)
      {
      }
   };

   public TestOcTree(double resolution, int treeDepth)
   {
      super(resolution, treeDepth);
   }

   public TestOcTree(TestOcTree other)
   {
      super(other);
   }

   public void insertNode(OcTreeKeyReadOnly key)
   {
      updateNodeInternal(key, doNothingRule, null);
   }

   public void insertNode(Point3DReadOnly coordinate)
   {
      updateNodeInternal(coordinate, doNothingRule, null);
   }

   public void fillRandomly(Random random, int numberOfLeavesToCreate)
   {
      while (getNumberOfLeafNodes() < numberOfLeavesToCreate)
      {
         OcTreeKey randomKey = new OcTreeKey(random, treeDepth, treeDepth);
         insertNode(randomKey);
      }
   }

   public void fillRandomlyWithinSphere(Random random, int numberOfLeavesToCreate, Point3DReadOnly sphereCenter, double sphereRadius)
   {
      while (getNumberOfLeafNodes() < numberOfLeavesToCreate)
      {
         Vector3D randomTranslation = new Vector3D(random.nextDouble() - 0.5, random.nextDouble() - 0.5, random.nextDouble() - 0.5);
         randomTranslation.scale(sphereRadius * random.nextDouble() / randomTranslation.length());
         Point3D randomCoordinateInSphere = new Point3D();
         randomCoordinateInSphere.add(sphereCenter, randomTranslation);
         insertNode(randomCoordinateInSphere);
      }
   }

   @Override
   protected Class<TestOcTreeNode> getNodeClass()
   {
      return TestOcTreeNode.class;
   }
}
