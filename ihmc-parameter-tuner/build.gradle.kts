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

   api("us.ihmc:ihmc-robot-data-logger:0.34.0")
   api("us.ihmc:ihmc-javafx-toolkit:17-0.22.12")
   api("us.ihmc:simulation-construction-set-utilities:0.25.3")

   var javaFXVersion = "17.0.8"
   api(ihmc.javaFXModule("base", javaFXVersion))
   api(ihmc.javaFXModule("controls", javaFXVersion))
   api(ihmc.javaFXModule("graphics", javaFXVersion))
   api(ihmc.javaFXModule("fxml", javaFXVersion))
}

testDependencies {
   api("us.ihmc:log-tools:0.6.5")
}
