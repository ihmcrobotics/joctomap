package us.ihmc.octoMap.exceptions;

public class PlanarRegionSegmentationException extends RuntimeException
{
   private static final long serialVersionUID = -5843579452186405801L;

   public PlanarRegionSegmentationException()
   {
   }

   public PlanarRegionSegmentationException(String message)
   {
      super(message);
   }
}
