plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.1"
   id("us.ihmc.ihmc-cd") version "1.24"
}

ihmc {
   group = "us.ihmc"
   version = "1.12.5"
   openSource = true
   maintainer = "Sylvain Bertrand"
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("org.apache.commons:commons-math3:3.6.1")
   api("org.apache.commons:commons-lang3:3.12.0")

   api("us.ihmc:euclid-geometry:0.20.0")
}

testDependencies {
}
