package us.ihmc.perception;

/**
 * Used with the ImageMessage camera model field.
 */
public enum CameraModel
{
   /**
    * Is of the pinhole camera model (See https://en.wikipedia.org/wiki/Pinhole_camera_model)
    */
   PINHOLE,
   /**
    * If the camera model is the Ouster camera model.
    * With the Ouster camera model, the vertical and horizontal fields of view
    * are used instead of the focal length and principal point parameters.
    */
   OUSTER,
   /**
    * Is equidistant fisheye camera model (See https://en.wikipedia.org/wiki/Fisheye_lens#Mapping_function)
    */
   EQUIDISTANT_FISHEYE,
   /**
    * Used for height maps.
    */
   ORTHOGRAPHIC;

   public static final CameraModel[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static CameraModel fromByte(byte ordinalByte)
   {
      return values[ordinalByte];
   }
}
