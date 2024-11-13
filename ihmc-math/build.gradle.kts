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
   api("us.ihmc:ihmc-matrix-library:0.19.1")
   api("us.ihmc:ihmc-robotics-tools:source")
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
