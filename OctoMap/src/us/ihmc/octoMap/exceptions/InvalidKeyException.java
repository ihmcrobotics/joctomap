package us.ihmc.octoMap.exceptions;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;

public class InvalidKeyException extends RuntimeException
{
   private static final long serialVersionUID = 2484570003328160671L;

   public InvalidKeyException(int invalidKey, int depth)
   {
      super("The key: " + invalidKey + " is invalid (at depth: " + depth + ").");
   }

   public InvalidKeyException(OcTreeKeyReadOnly invalidKey, int depth)
   {
      super("The key: " + invalidKey + " is invalid (at depth: " + depth + ").");
   }

   public InvalidKeyException(int invalidKey)
   {
      super("The key: " + invalidKey + " is invalid.");
   }
}
