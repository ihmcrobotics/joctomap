plugins {
   id("us.ihmc.ihmc-build") version "0.20.1"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.8"
}

ihmc {
   group = "us.ihmc"
   version = "1.9.0"
   openSource = true
   maintainer = "Sylvain Bertrand"
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.apache.commons:commons-math3:3.3")
   api("org.apache.commons:commons-lang3:3.9")
   api("org.ejml:dense64:0.30")
   api("us.ihmc:euclid-geometry:0.14.1")
}

testDependencies {
}
