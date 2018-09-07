plugins {
   id("us.ihmc.ihmc-build") version "0.10.5"
}

ihmc {
   group = "us.ihmc"
   version = "1.6.7"
   openSource = true
   maintainer = "Sylvain Bertrand"
   
   configureDependencyResolution()
   configurePublications()
}

dependencies {
   compile("us.ihmc:euclid-core:0.4.6")
   compile("org.apache.commons:commons-math3:3.3")
   compile("org.apache.commons:commons-lang3:3.3")
   compile("org.ejml:dense64:0.30")
}
