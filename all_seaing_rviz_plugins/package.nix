# Automatically generated by: ros2nix --
{
  lib,
  buildRosPackage,
  all-seaing-interfaces,
  ament-cmake-auto,
  pluginlib,
  rviz-common,
  rviz-default-plugins,
  rviz-rendering,
}:
buildRosPackage rec {
  pname = "ros-rolling-all-seaing-rviz-plugins";
  version = "0.0.0";

  src = ./.;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake-auto ];
  propagatedBuildInputs = [
    all-seaing-interfaces
    pluginlib
    rviz-common
    rviz-default-plugins
    rviz-rendering
  ];
  nativeBuildInputs = [ ament-cmake-auto ];

  meta = {
    description = "RViz plugins for all_seaing_vehicle";
    license = with lib.licenses; [ mit ];
  };
}