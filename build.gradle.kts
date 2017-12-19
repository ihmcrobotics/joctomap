plugins {
   id("us.ihmc.ihmc-build") version "0.11.0"
}

ihmc {
   group = "us.ihmc"
   version = "1.6.8"
   openSource = true
   maintainer = "Sylvain Bertrand"
   
   configureDependencyResolution()
   configurePublications()
}

dependencies {
   compile("us.ihmc:euclid:0.7.5")
   compile("org.apache.commons:commons-math3:3.3")
   compile("org.apache.commons:commons-lang3:3.3")
   compile("org.ejml:dense64:0.30")
}
