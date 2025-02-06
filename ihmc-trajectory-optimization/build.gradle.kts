plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.log-tools-plugin") version "0.6.4"
}

ihmc {
   loadProductProperties("../group.product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("net.sf.trove4j:trove4j:3.0.3")
   api("us.ihmc:ihmc-commons:0.35.1")
   api("us.ihmc:ihmc-matrix-library:0.19.3")
}

testDependencies {
   api("org.ejml:ejml-ddense:0.39")
}

examplesDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:euclid-frame:0.22.3")
   api("us.ihmc:euclid-geometry:0.22.3")
   api("us.ihmc:ihmc-yovariables:0.13.5")
   api("us.ihmc:simulation-construction-set:0.25.3")
}

examplesTestDependencies{
   api(ihmc.sourceSetProject("examples"))
}

