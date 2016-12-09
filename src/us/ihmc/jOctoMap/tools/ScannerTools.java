package us.ihmc.jOctoMap.tools;

import java.util.NoSuchElementException;
import java.util.Scanner;

public class ScannerTools
{
   public static double readNextDouble(Scanner scanner, double defaultValue)
   {
      try
      {
         while (!scanner.hasNextDouble())
            scanner.next();
         return scanner.nextDouble();
      }
      catch (NoSuchElementException e)
      {
         return defaultValue;
      }
   }

   public static int readNextInt(Scanner scanner, int defaultValue)
   {
      try
      {
         while (!scanner.hasNextInt())
            scanner.next();
         return scanner.nextInt();
      }
      catch (NoSuchElementException e)
      {
         return defaultValue;
      }
   }

   public static boolean readNextBoolean(Scanner scanner, boolean defaultValue)
   {
      try
      {
         while (!scanner.hasNextBoolean())
            scanner.next();
         return scanner.nextBoolean();
      }
      catch (NoSuchElementException e)
      {
         return defaultValue;
      }
   }
}
