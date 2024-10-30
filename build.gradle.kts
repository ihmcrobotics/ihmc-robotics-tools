plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   group = "us.ihmc"
   version = "0.0.0"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-math"
   openSource = true
   maintainer = "Robert Griffin"
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-matrix-library:0.19.0")
   api("us.ihmc:ihmc-robotics-tools:0.1.0")
   api("org.ejml:ejml-simple:0.39")
   api("gov.nist.math:jama:1.0.3")
   api("us.ihmc:ihmc-native-library-loader:2.0.3")
}

linearAlgebraDependencies {
   api(ihmc.sourceSetProject("main"))
}

linearDynamicSystemsDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("linear-algebra"))
   api("org.jfree:jfreechart:1.0.19")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("linear-algebra"))
   api(ihmc.sourceSetProject("linear-dynamic-systems"))
}
