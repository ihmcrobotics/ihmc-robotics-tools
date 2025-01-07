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
   api("us.ihmc:euclid-frame:0.22.2")
   api("us.ihmc:ihmc-commons:0.35.0")

   val opencvVersion = "4.7.0-1.5.9"
   api("org.bytedeco:opencv:$opencvVersion")
   api("org.bytedeco:opencv:$opencvVersion:linux-x86_64")
   api("org.bytedeco:opencv:$opencvVersion:linux-arm64")
   api("org.bytedeco:opencv:$opencvVersion:windows-x86_64")
   api("org.bytedeco:opencv:$opencvVersion:linux-x86_64-gpu")
   api("org.bytedeco:opencv:$opencvVersion:windows-x86_64-gpu")
   val ffmpegVersion = "6.0-1.5.9"
   api("org.bytedeco:ffmpeg:$ffmpegVersion")
   api("org.bytedeco:ffmpeg:$ffmpegVersion:linux-x86_64")
   api("org.bytedeco:ffmpeg:$ffmpegVersion:linux-arm64")
   api("org.bytedeco:ffmpeg:$ffmpegVersion:windows-x86_64")
}
