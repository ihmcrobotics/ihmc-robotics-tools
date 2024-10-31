plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   group = "us.ihmc"
   version = "0.1.0"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-robotics-tools"
   openSource = true
   maintainer = "Robert Griffin rgriffin@ihmc.org"
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-commons:0.34.0")
   api("us.ihmc:euclid-frame:0.22.2")
   api("us.ihmc:ihmc-yovariables:0.13.3")
   api("us.ihmc:ihmc-yovariables-filters:0.13.3")
   api("us.ihmc:ihmc-graphics-description:0.25.2")
}

jointKinematicsDependencies {
   api(ihmc.sourceSetProject("main"))
}

visualizersDependencies{
   api(ihmc.sourceSetProject("main"))
   api("jgraph:jgraph:5.13.0.0")
}

testDependencies {
   api(ihmc.sourceSetProject("joint-kinematics"))
   api("org.apache.commons:commons-math3:3.6.1")
   api("us.ihmc:ihmc-commons-testing:0.34.0")
}
