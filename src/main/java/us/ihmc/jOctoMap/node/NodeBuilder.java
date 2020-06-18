package us.ihmc.jOctoMap.node;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;

public class NodeBuilder<NODE>
{
   private final Constructor<NODE> emptyConstructor;

   public NodeBuilder(Class<NODE> nodeClass)
   {
      // Trying to get an empty constructor from clazz
      try
      {
         emptyConstructor = nodeClass.getConstructor();
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         throw new RuntimeException("Could not find an empty constructor for the node class: " + nodeClass.getSimpleName());
      }
   }

   public NODE createNode()
   {
      NODE newNode = null;

      try
      {
         newNode = emptyConstructor.newInstance();
      }
      catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         e.printStackTrace();
         throw new RuntimeException("Something went wrong the empty constructor implemented in the node class: "
               + emptyConstructor.getDeclaringClass().getSimpleName());
      }

      return newNode;
   }
}