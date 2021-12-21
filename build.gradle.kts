plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.5"
   id("us.ihmc.ihmc-cd") version "1.21"
}

ihmc {
   group = "us.ihmc"
   version = "1.12.1"
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

   api("us.ihmc:euclid-geometry:0.17.0")
}

testDependencies {
}
