plugins {
   id("us.ihmc.ihmc-build") version "0.22.0"
   id("us.ihmc.ihmc-ci") version "7.1"
   id("us.ihmc.ihmc-cd") version "1.16"
}

ihmc {
   group = "us.ihmc"
   version = "1.11.0"
   openSource = true
   maintainer = "Sylvain Bertrand"
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("org.apache.commons:commons-math3:3.3")
   api("org.apache.commons:commons-lang3:3.11")

   api("us.ihmc:euclid-geometry:0.15.1")
}

testDependencies {
}
