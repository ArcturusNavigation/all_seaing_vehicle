// Node to overlay pointcloud returns over image

#include "pointcloud_image_fusion/pointcloud_image_overlay.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, __APP_NAME__);

    PclImageOverlay app;

    app.Run();

    return 0;
}
