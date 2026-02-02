plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.5"
}

ihmc {
   loadProductProperties("../group.product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-commons:0.35.1")
   api("us.ihmc:euclid-frame:0.22.5")
   api("us.ihmc:ihmc-yovariables:0.13.7")
   api("us.ihmc:ihmc-yovariables-filters:0.13.7")
   api("us.ihmc:ihmc-graphics-description:0.26.2")
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
   api("us.ihmc:ihmc-commons-testing:0.35.1")
   api("com.google.guava:guava:18.0")
}
