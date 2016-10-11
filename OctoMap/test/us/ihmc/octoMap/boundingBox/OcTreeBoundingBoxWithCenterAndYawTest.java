package us.ihmc.octoMap.boundingBox;

import static org.junit.Assert.*;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.octoMap.key.OcTreeKey;

public class OcTreeBoundingBoxWithCenterAndYawTest 
{
	double resolution = 0.025;
	int treeDepth = 16;
	double yaw = Math.PI/2;
	Point3d offset = new Point3d(-150.0, 150.0, 0.0);
	
	Point3d minCoordinate = new Point3d(0.0, 0.0, 0.0);
	Point3d maxCoordinate = new Point3d(100.0, 200.0, 100.0);
	OcTreeKey minKey = new OcTreeKey(32768,32768,32768);
	OcTreeKey maxKey = new OcTreeKey(36768, 40768, 36768);
	
	@Test
	public void isPointInRotatedBoundingBoxTest()
	{
		Point3d pointA = new Point3d(-200.0, 200.0, 50.0);
		OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(minCoordinate, maxCoordinate, resolution, treeDepth);
		boundingBox.setYaw(yaw);
		boundingBox.setOffset(offset, resolution, treeDepth);
		
		assertTrue(boundingBox.isInBoundingBox(pointA));
	}
	
	@Test 
	public void isKeyInSimpleBoundingBoxTest()
	{
		OcTreeKey keyA = new OcTreeKey(34000, 39000, 34000);
		OcTreeSimpleBoundingBox boundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
		boundingBox.update(resolution, treeDepth);
		
		assertTrue(boundingBox.isInBoundingBox(keyA));
	}
	
	@Test 
	public void isKeyInRotatedBoundingBoxTest()
	{
		//OcTreeKey key = new OcTreeKey(29940, 35596, 34768); //45 degrees yaw 
		OcTreeKey key = new OcTreeKey(29304, 34768, 34768); //60 degrees yaw 
		OcTreeSimpleBoundingBox simpleBoundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
		simpleBoundingBox.update(resolution, treeDepth);
		
		OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(simpleBoundingBox, resolution, treeDepth);
		boundingBox.setYaw(Math.PI/3);

		assertTrue(boundingBox.isInBoundingBox(key));
	}
	
	@Test
	public void isKeyInRotatedAndOffsetBoundingBoxTest()
	{
		OcTreeKey key = new OcTreeKey(23940, 41596, 34768); //45 degrees yaw and offset (-150.0, 150.0, 0.0)
		OcTreeSimpleBoundingBox simpleBoundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
		simpleBoundingBox.update(resolution, treeDepth);
		
		OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(simpleBoundingBox, resolution, treeDepth);
		boundingBox.setYaw(Math.PI/4);
		boundingBox.setOffset(offset, resolution, treeDepth);

		assertTrue(boundingBox.isInBoundingBox(key));
	}
	
}
