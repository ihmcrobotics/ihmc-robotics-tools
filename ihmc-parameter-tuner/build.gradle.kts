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
   api("net.sf.trove4j:trove4j:3.0.3")
   api("org.apache.commons:commons-math3:3.6.1")

   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.11")
   api("us.ihmc:scs2-session-visualizer-jfx:17-0.32.0")
}

testDependencies {
   api("us.ihmc:log-tools:0.6.5")
}
